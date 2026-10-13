#include "SharpaManusClient.hpp"
#include "ManusSDKTypes.h"
#include "ClientLogging.hpp"
#include <iostream>
#include <thread>
#include <array>
#include <Eigen/Dense>
#include <zmq.hpp>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "sharpa_hand.pb.h"
#include "ClientPlatformSpecific.hpp"

#define M_PI 3.14159265358979323846

using ManusSDK::ClientLog;

namespace
{
	std::filesystem::path FindLatestCoreLiteSettingsFile()
	{
		const char* t_Home = std::getenv("HOME");
		if (t_Home == nullptr)
		{
			return {};
		}

		const std::filesystem::path t_CoreDir =
			std::filesystem::path(t_Home) / ".config" / "Manus" / "Core 3";
		if (!std::filesystem::is_directory(t_CoreDir))
		{
			return {};
		}

		std::filesystem::path t_Latest;
		std::filesystem::file_time_type t_LatestTime{};
		bool t_Found = false;

		for (const auto& t_Entry : std::filesystem::directory_iterator(t_CoreDir))
		{
			if (!t_Entry.is_regular_file())
			{
				continue;
			}

			const std::string t_Name = t_Entry.path().filename().string();
			if (t_Name.rfind("CoreLite.Settings.", 0) != 0)
			{
				continue;
			}
			if (t_Name.find(".Calibrations.") != std::string::npos
				|| t_Name.find(".Devices.") != std::string::npos)
			{
				continue;
			}

			const auto t_Time = t_Entry.last_write_time();
			if (!t_Found || t_Time > t_LatestTime)
			{
				t_Found = true;
				t_Latest = t_Entry.path();
				t_LatestTime = t_Time;
			}
		}

		return t_Latest;
	}

	bool IsCoreAutoAssignmentEnabledInConfig()
	{
		const std::filesystem::path t_SettingsPath = FindLatestCoreLiteSettingsFile();
		if (t_SettingsPath.empty())
		{
			return false;
		}

		std::ifstream t_File(t_SettingsPath);
		if (!t_File)
		{
			return false;
		}

		const std::string t_Content(
			(std::istreambuf_iterator<char>(t_File)),
			std::istreambuf_iterator<char>());
		const std::size_t t_Pos = t_Content.find("\"autoAssignment\"");
		if (t_Pos == std::string::npos)
		{
			return false;
		}

		const std::string t_Snippet = t_Content.substr(
			t_Pos,
			std::min<std::size_t>(64, t_Content.size() - t_Pos));
		return t_Snippet.find("true") != std::string::npos;
	}

	void AppendPoseToMocap(
		proto::MocapKeypoints& p_Msg,
		Side p_Side,
		const Eigen::Vector3f& p_Pos,
		const Eigen::Quaternionf& p_Rot)
	{
		proto::Pose* t_Pose = nullptr;
		if (p_Side == Side::Side_Left)
		{
			t_Pose = p_Msg.add_left_mocap_pose();
		}
		else if (p_Side == Side::Side_Right)
		{
			t_Pose = p_Msg.add_right_mocap_pose();
		}
		if (t_Pose == nullptr)
		{
			return;
		}

		proto::Point* t_Position = t_Pose->mutable_position();
		proto::Quaternion* t_Orientation = t_Pose->mutable_orientation();
		t_Position->set_x(p_Pos.x());
		t_Position->set_y(p_Pos.y());
		t_Position->set_z(p_Pos.z());
		t_Orientation->set_w(p_Rot.w());
		t_Orientation->set_x(p_Rot.x());
		t_Orientation->set_y(p_Rot.y());
		t_Orientation->set_z(p_Rot.z());
	}

	bool AppendSkeletonNodesToMocap(
		proto::MocapKeypoints& p_Msg,
		Side p_Side,
		const std::vector<SkeletonNode>& p_Nodes)
	{
		if (p_Nodes.empty())
		{
			return false;
		}

		const auto& t_RootNode = p_Nodes[0];
		Eigen::Vector3f t_RootPos(
			t_RootNode.transform.position.x,
			t_RootNode.transform.position.y,
			t_RootNode.transform.position.z);
		Eigen::Quaternionf t_RootRot(
			t_RootNode.transform.rotation.w,
			t_RootNode.transform.rotation.x,
			t_RootNode.transform.rotation.y,
			t_RootNode.transform.rotation.z);
		t_RootRot.normalize();

		const float t_Angle = -M_PI / 2.0f;
		const Eigen::Quaternionf t_YRotation90(std::cos(t_Angle / 2.0f), 0.0f, std::sin(t_Angle / 2.0f), 0.0f);
		const int t_NodeLimit = static_cast<int>(p_Nodes.size());

		// Publish Manus raw order 0..24 (no finger remap).
		for (int j = 0; j < 25; j++)
		{
			const int t_OriginalIndex = std::min(j, t_NodeLimit - 1);
			const auto& t_Node = p_Nodes[t_OriginalIndex];
			Eigen::Vector3f t_NodePos(
				t_Node.transform.position.x,
				t_Node.transform.position.y,
				t_Node.transform.position.z);
			Eigen::Quaternionf t_NodeRot(
				t_Node.transform.rotation.w,
				t_Node.transform.rotation.x,
				t_Node.transform.rotation.y,
				t_Node.transform.rotation.z);
			t_NodeRot.normalize();

			Eigen::Vector3f t_RelativePos = t_RootRot.inverse() * (t_NodePos - t_RootPos);
			Eigen::Quaternionf t_RelativeRot = t_RootRot.inverse() * t_NodeRot;
			Eigen::Vector3f t_RotatedPos = t_YRotation90 * t_RelativePos;
			Eigen::Quaternionf t_RotatedRot = t_YRotation90 * t_RelativeRot * t_YRotation90.inverse();
			AppendPoseToMocap(p_Msg, p_Side, t_RotatedPos, t_RotatedRot);
		}

		return true;
	}
}

Side SDKClient::ResolveSkeletonSideForGlove(uint32_t p_GloveId)
{
	if (s_Instance == nullptr || p_GloveId == 0)
	{
		return Side::Side_Invalid;
	}

	if (s_Instance->m_FirstLeftGloveID != UINT32_MAX && p_GloveId == s_Instance->m_FirstLeftGloveID)
	{
		return Side::Side_Left;
	}
	if (s_Instance->m_FirstRightGloveID != UINT32_MAX && p_GloveId == s_Instance->m_FirstRightGloveID)
	{
		return Side::Side_Right;
	}

	return ResolveGloveSide(p_GloveId);
}

bool SDKClient::IsHandCacheFresh(const CachedHandSkeleton& p_Cache)
{
	if (!p_Cache.valid || p_Cache.nodes.empty())
	{
		return false;
	}

	const auto t_Now = std::chrono::steady_clock::now();
	const auto t_AgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(t_Now - p_Cache.updatedAt).count();
	return t_AgeMs < k_HandSkeletonCacheMaxAgeMs;
}

void SDKClient::UpdateHandSkeletonCache(Side p_Side, std::vector<SkeletonNode> p_Nodes)
{
	if (p_Side != Side::Side_Left && p_Side != Side::Side_Right)
	{
		return;
	}
	if (p_Nodes.empty())
	{
		return;
	}

	std::lock_guard<std::mutex> t_Lock(m_HandSkeletonCacheMutex);
	CachedHandSkeleton& t_Cache =
		(p_Side == Side::Side_Left) ? m_LeftHandSkeletonCache : m_RightHandSkeletonCache;
	t_Cache.nodes = std::move(p_Nodes);
	t_Cache.updatedAt = std::chrono::steady_clock::now();
	t_Cache.valid = true;
}

SDKClient* SDKClient::s_Instance = nullptr;

SDKClient::SDKClient()
{
	s_Instance = this;
	m_FrameId = 0;
}

SDKClient::~SDKClient()
{
	StopHapticFeedbackThread();
	s_Instance = nullptr;
}

ClientReturnCode SDKClient::Initialize()
{
	if (!PlatformSpecificInitialization())
	{
		return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificInitialization;
	}

	m_ZmqPubContext = std::make_shared<zmq::context_t>(1);
	m_ZmqPublisher = std::make_shared<zmq::socket_t>(*m_ZmqPubContext, ZMQ_PUB);
	
	// Set send high water mark to prevent memory overflow
	int sndhwm = 10;  // Send high water mark
	m_ZmqPublisher->set(zmq::sockopt::sndhwm, sndhwm);
	int linger = 0;   // Close immediately
	m_ZmqPublisher->set(zmq::sockopt::linger, linger);
	
	m_ZmqPublisher->bind(m_ZmqHost);
	ClientLog::print("ZMQ Mocap PUB bound: {}", m_ZmqHost);

	const ClientReturnCode t_IntializeResult = InitializeSDK();
	if (t_IntializeResult != ClientReturnCode::ClientReturnCode_Success)
	{
		ClientLog::error("Failed to initialize the SDK. Are you sure the correct ManusSDKLibary is used?");
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	m_HapticConfig = HapticConfig::LoadFromEnvOrDefault();
	m_HapticHandler.ApplyConfig(m_HapticConfig);

	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::Connect()
{
	bool t_ConnectLocally = m_ConnectionType == ConnectionType::ConnectionType_Local;
	SDKReturnCode t_StartResult = CoreSdk_LookForHosts(1, t_ConnectLocally);
	if (t_StartResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	uint32_t t_NumberOfHostsFound = 0;
	SDKReturnCode t_NumberResult = CoreSdk_GetNumberOfAvailableHostsFound(&t_NumberOfHostsFound);
	if (t_NumberResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	if (t_NumberOfHostsFound == 0)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	std::unique_ptr<ManusHost[]> t_AvailableHosts; 
	t_AvailableHosts.reset(new ManusHost[t_NumberOfHostsFound]);

	SDKReturnCode t_HostsResult = CoreSdk_GetAvailableHostsFound(t_AvailableHosts.get(), t_NumberOfHostsFound);
	if (t_HostsResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	uint32_t t_HostSelection = 0;
	if (!t_ConnectLocally && t_NumberOfHostsFound > 1)
	{
		ClientLog::print("Select which host you want to connect to (and press enter to submit)");
		for (size_t i = 0; i < t_NumberOfHostsFound; i++)
		{
			auto t_HostInfo = t_AvailableHosts[i];
			ClientLog::print("[{}] hostname: {} IP address: {}, version {}.{}.{}", i + 1, t_HostInfo.hostName, t_HostInfo.ipAddress, t_HostInfo.manusCoreVersion.major, t_HostInfo.manusCoreVersion.minor, t_HostInfo.manusCoreVersion.patch);
		}
		uint32_t t_HostSelectionInput = 0;
		std::cin >> t_HostSelectionInput;
		if (t_HostSelectionInput <= 0 || t_HostSelectionInput > t_NumberOfHostsFound)
			return ClientReturnCode::ClientReturnCode_FailedToConnect;

		t_HostSelection = t_HostSelectionInput - 1;
	}

	SDKReturnCode t_ConnectResult = CoreSdk_ConnectToHost(t_AvailableHosts[t_HostSelection]);

	if (t_ConnectResult == SDKReturnCode::SDKReturnCode_NotConnected)
	{
		return ClientReturnCode::ClientReturnCode_FailedToConnect;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::Run()
{
	ClearConsole();

	// first loop until we get a connection
	m_ConnectionType == ConnectionType::ConnectionType_Integrated ?
		ClientLog::print("SDK client is running in integrated mode.") :
		ClientLog::print("SDK client is connecting to MANUS Core. (make sure it is running)");

	while (Connect() != ClientReturnCode::ClientReturnCode_Success)
	{
		// not yet connected. wait
		ClientLog::print("minimal client could not connect.trying again in a second.");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// Subscribe WAVE tactile force stream from retargeting (HapticFeedback @ 30Hz).
	StartHapticFeedbackThread();

	EnsureAutoUserAssignment();

	if (m_ConnectionType != ConnectionType::ConnectionType_Integrated)
		ClientLog::print("SDK client is connected, setting up skeletons.");

	// Legacy Pro client always used HandMotion_None; do not switch to Auto.
	const SDKReturnCode t_HandMotionResult = CoreSdk_SetRawSkeletonHandMotion(HandMotion_None);
	if (t_HandMotionResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to set hand motion mode. The value returned was {}.", (int32_t)t_HandMotionResult);
	}
	
	while (m_Running)
	{
		UpdateInput();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		
		// Check if left hand calibration needs to be loaded
		if (!m_LeftGloveCalibrationLoaded && m_FirstLeftGloveID != UINT32_MAX)
		{
			LoadGloveCalibration(m_FirstLeftGloveID, "Calibration_left.mcal");
			m_LeftGloveCalibrationLoaded = true;
		}
			
		// Check if right hand calibration needs to be loaded
		if (!m_RightGloveCalibrationLoaded && m_FirstRightGloveID != UINT32_MAX)
		{
			LoadGloveCalibration(m_FirstRightGloveID, "Calibration_right.mcal");
			m_RightGloveCalibrationLoaded = true;
		}
		
		// Apply tiered PWM from latest WAVE force magnitudes (50Hz tick).
		ProcessWaveHapticPowers();
		ProcessGlovePairing();

		if (GetKeyDown('H')) // manual motor test (index finger pulse)
		{
			ClientLog::print("H pressed: running haptics test...");
			RunHapticsTest();
		}

		if (GetKeyDown('T')) // toggle WAVE->Manus haptic path
		{
			ToggleWaveHapticsEnabled();
		}

		if (GetKeyDown('D')) // toggle haptic debug print (force N + tier per finger)
		{
			ToggleHapticsDebugPrint();
		}

		if (GetKeyDown('J')) // synthetic tier test without ZMQ / WAVE
		{
			ClientLog::print("J pressed: synthetic Low/Mid/High tier test on index finger...");
			RunSyntheticHapticTierTest();
		}

		if (GetKeyDown(' ')) // press space to exit
		{
			m_Running = false;
		}
	}
	
	return ClientReturnCode::ClientReturnCode_Success;
}


ClientReturnCode SDKClient::ShutDown()
{
	StopHapticFeedbackThread();
	StopAllHaptics();

	const SDKReturnCode t_Result = CoreSdk_ShutDown();
	if (t_Result != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to shut down the SDK wrapper. The value returned was {}.", (int32_t)t_Result);
		return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;
	}

	if (!PlatformSpecificShutdown())
	{
		return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificShutdown;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

void SDKClient::OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_SkeletonStreamInfo)
{
	if (s_Instance == nullptr || p_SkeletonStreamInfo == nullptr)
	{
		return;
	}

	const auto t_IngestSkeletonIndex = [&](uint32_t p_Index) -> bool {
		RawSkeletonInfo t_Info{};
		RawSkeletonInfo_Init(&t_Info);
		if (CoreSdk_GetRawSkeletonInfo(p_Index, &t_Info) != SDKReturnCode::SDKReturnCode_Success)
		{
			return false;
		}
		if (t_Info.gloveId == 0 || t_Info.nodesCount == 0)
		{
			return false;
		}

		const Side t_Side = ResolveSkeletonSideForGlove(t_Info.gloveId);
		if (t_Side == Side::Side_Invalid)
		{
			return false;
		}

		std::vector<SkeletonNode> t_Nodes(t_Info.nodesCount);
		if (CoreSdk_GetRawSkeletonData(p_Index, t_Nodes.data(), t_Info.nodesCount)
			!= SDKReturnCode::SDKReturnCode_Success)
		{
			return false;
		}
		if (t_Nodes.empty())
		{
			return false;
		}

		s_Instance->UpdateHandSkeletonCache(t_Side, std::move(t_Nodes));
		return true;
	};

	for (uint32_t i = 0; i < p_SkeletonStreamInfo->skeletonsCount; i++)
	{
		t_IngestSkeletonIndex(i);
	}

	const bool t_DualGlovesExpected =
		s_Instance->m_FirstLeftGloveID != UINT32_MAX
		&& s_Instance->m_FirstRightGloveID != UINT32_MAX;
	if (t_DualGlovesExpected && p_SkeletonStreamInfo->skeletonsCount == 1)
	{
		// Core often reports skeletonsCount=1 while both gloves are active; probe slot 1.
		t_IngestSkeletonIndex(1);
	}

	proto::MocapKeypoints mocap_keypoints_msg;

	ManusTimestamp t_Timestamp = p_SkeletonStreamInfo->publishTime;
	const uint64_t timestamp = t_Timestamp.time;
	const uint64_t sec = timestamp / 1000000000;
	const uint64_t nanosec = timestamp % 1000000000;

	const std::string id = std::to_string(s_Instance->m_FrameId);
	s_Instance->m_FrameId++;

	proto::Header* header_msg = mocap_keypoints_msg.mutable_header();
	proto::Stamp* stamp_msg = header_msg->mutable_stamp();
	stamp_msg->set_sec(static_cast<int32_t>(sec));
	stamp_msg->set_nanosec(static_cast<int32_t>(nanosec));
	header_msg->set_frame_id(id);

	bool t_PublishedLeft = false;
	bool t_PublishedRight = false;
	{
		std::lock_guard<std::mutex> t_Lock(s_Instance->m_HandSkeletonCacheMutex);
		if (IsHandCacheFresh(s_Instance->m_LeftHandSkeletonCache))
		{
			t_PublishedLeft = AppendSkeletonNodesToMocap(
				mocap_keypoints_msg,
				Side::Side_Left,
				s_Instance->m_LeftHandSkeletonCache.nodes);
		}
		if (IsHandCacheFresh(s_Instance->m_RightHandSkeletonCache))
		{
			t_PublishedRight = AppendSkeletonNodesToMocap(
				mocap_keypoints_msg,
				Side::Side_Right,
				s_Instance->m_RightHandSkeletonCache.nodes);
		}
	}

	if (!t_PublishedLeft && !t_PublishedRight)
	{
		return;
	}

	s_Instance->MaybeLogHandOutputStatus(t_PublishedLeft, t_PublishedRight);

	std::string serialized_data;
	mocap_keypoints_msg.SerializeToString(&serialized_data);
	zmq::message_t zmq_msg(serialized_data.size());
	memcpy(zmq_msg.data(), serialized_data.data(), serialized_data.size());
	s_Instance->m_ZmqPublisher->send(zmq_msg, zmq::send_flags::none);
}

bool SDKClient::VibrateGloveUnchecked(uint32_t p_GloveId, const float p_Powers[NUM_FINGERS_ON_HAND])
{
	if (p_GloveId == 0)
	{
		return false;
	}

	bool t_AllZero = true;
	for (int i = 0; i < NUM_FINGERS_ON_HAND; ++i)
	{
		if (p_Powers[i] > 0.001f)
		{
			t_AllZero = false;
			break;
		}
	}

	std::chrono::high_resolution_clock::time_point* t_LastCommandTime = nullptr;
	if (p_GloveId == m_LeftHapticGloveID)
	{
		t_LastCommandTime = &m_LastLeftHapticsCommandTime;
	}
	else if (p_GloveId == m_RightHapticGloveID)
	{
		t_LastCommandTime = &m_LastRightHapticsCommandTime;
	}

	const auto t_Now = std::chrono::high_resolution_clock::now();
	if (!t_AllZero && t_LastCommandTime != nullptr)
	{
		const auto t_Ms =
			std::chrono::duration_cast<std::chrono::milliseconds>(t_Now - *t_LastCommandTime).count();
		if (t_Ms < static_cast<long long>(m_HapticConfig.minMsBetweenHapticCommands))
		{
			return false;
		}
	}
	if (t_LastCommandTime != nullptr)
	{
		*t_LastCommandTime = t_Now;
	}

	const SDKReturnCode t_Result = CoreSdk_VibrateFingersForGlove(p_GloveId, p_Powers);
	if (t_Result != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("VibrateFingersForGlove failed glove 0x{} rc={}", p_GloveId, (int32_t)t_Result);
		return false;
	}
	return true;
}

void SDKClient::VibrateGlove(uint32_t p_GloveId, const float p_Powers[NUM_FINGERS_ON_HAND])
{
	VibrateGloveUnchecked(p_GloveId, p_Powers);
}

void SDKClient::StopAllHaptics()
{
	const float t_Zero[NUM_FINGERS_ON_HAND] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	if (m_LeftHapticGloveID != 0)
	{
		CoreSdk_VibrateFingersForGlove(m_LeftHapticGloveID, t_Zero);
	}
	if (m_RightHapticGloveID != 0)
	{
		CoreSdk_VibrateFingersForGlove(m_RightHapticGloveID, t_Zero);
	}
}

void SDKClient::RunHapticsTest()
{
	if (!m_LeftGloveIsHaptics && !m_RightGloveIsHaptics)
	{
		ClientLog::warn("Haptics test skipped: no haptic glove (Pro/Pro Precision has no motors; need Metaglove Pro Haptics).");
		return;
	}

	ClientLog::print("Haptics test: index finger pulse (Thumb,Index,Middle,Ring,Pinky order). Press H again to repeat.");

	// Thumb, Index, Middle, Ring, Pinky
	const float t_Pulse[NUM_FINGERS_ON_HAND] = {0.0f, 0.8f, 0.0f, 0.0f, 0.0f};
	if (m_LeftGloveIsHaptics && m_LeftHapticGloveID != 0)
	{
		VibrateGlove(m_LeftHapticGloveID, t_Pulse);
	}
	if (m_RightGloveIsHaptics && m_RightHapticGloveID != 0)
	{
		VibrateGlove(m_RightHapticGloveID, t_Pulse);
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(300));
	StopAllHaptics();
	ClientLog::print("Haptics test finished.");
}

void SDKClient::RunSyntheticHapticTierTest()
{
	if (!m_LeftGloveIsHaptics && !m_RightGloveIsHaptics)
	{
		ClientLog::warn("Synthetic tier test skipped: need Metaglove Pro Haptics glove.");
		return;
	}

	const bool t_WasEnabled = m_HapticHandler.IsEnabled();
	m_HapticHandler.SetEnabled(true);
	m_HapticHandler.SetDebugPrint(true);

	m_HapticHandler.RunSyntheticTierTest(m_HapticConfig.syntheticStepDurationSec);

	const float t_Left[NUM_FINGERS_ON_HAND] = {0};
	const float t_Right[NUM_FINGERS_ON_HAND] = {0};
	m_HapticHandler.UpdateForceMagnitudes(t_Left, t_Right);

	for (int i = 0; i < 30; ++i)
	{
		ProcessWaveHapticPowers();
		const int t_PwmHz = std::max(1, m_HapticConfig.pwmTickHz);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000 / t_PwmHz));
	}

	StopAllHaptics();
	m_HapticHandler.SetEnabled(t_WasEnabled);
}

void SDKClient::StartHapticFeedbackThread()
{
	if (m_HapticThreadRunning.load())
	{
		return;
	}

	m_ZmqHapticContext = std::make_shared<zmq::context_t>(1);
	m_ZmqHapticSubscriber = std::make_shared<zmq::socket_t>(*m_ZmqHapticContext, ZMQ_SUB);
	m_ZmqHapticSubscriber->set(zmq::sockopt::rcvtimeo, 100);
	m_ZmqHapticSubscriber->set(zmq::sockopt::rcvhwm, 5);
	m_ZmqHapticSubscriber->connect(m_HapticSubAddress);
	m_ZmqHapticSubscriber->set(zmq::sockopt::subscribe, "");

	m_HapticThreadRunning.store(true);
	m_HapticFeedbackThread = std::thread(&SDKClient::HapticFeedbackThreadFunc, this);

	ClientLog::print(
		"WAVE haptic SUB connected: {} (expect retargeting -haptics PUB). Keys: T=toggle, D=debug, J=synth test",
		m_HapticSubAddress);
}

void SDKClient::StopHapticFeedbackThread()
{
	m_HapticThreadRunning.store(false);
	if (m_HapticFeedbackThread.joinable())
	{
		m_HapticFeedbackThread.join();
	}
	m_ZmqHapticSubscriber.reset();
	m_ZmqHapticContext.reset();
}

void SDKClient::HapticFeedbackThreadFunc()
{
	// Background recv: decouple ZMQ from main loop; PWM runs in ProcessWaveHapticPowers().
	while (m_HapticThreadRunning.load())
	{
		if (!m_ZmqHapticSubscriber)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			continue;
		}

		zmq::message_t t_Msg;
		const zmq::recv_result_t t_Result = m_ZmqHapticSubscriber->recv(t_Msg, zmq::recv_flags::none);
		if (!t_Result.has_value())
		{
			continue;
		}

		proto::HapticFeedback t_Feedback;
		if (!t_Feedback.ParseFromArray(t_Msg.data(), static_cast<int>(t_Msg.size())))
		{
			ClientLog::warn("[Haptics] Failed to parse HapticFeedback protobuf");
			continue;
		}

		float t_Left[NUM_FINGERS_ON_HAND] = {0};
		float t_Right[NUM_FINGERS_ON_HAND] = {0};
		const int t_LeftCount = std::min(static_cast<int>(t_Feedback.left_force_mag_size()), NUM_FINGERS_ON_HAND);
		const int t_RightCount = std::min(static_cast<int>(t_Feedback.right_force_mag_size()), NUM_FINGERS_ON_HAND);
		for (int i = 0; i < t_LeftCount; ++i)
		{
			t_Left[i] = t_Feedback.left_force_mag(i);
		}
		for (int i = 0; i < t_RightCount; ++i)
		{
			t_Right[i] = t_Feedback.right_force_mag(i);
		}
		m_HapticHandler.UpdateForceMagnitudes(t_Left, t_Right);
	}
}

void SDKClient::ProcessWaveHapticPowers()
{
	// Rate-limit PWM application (~50Hz); actual motor commands still respect 20ms SDK spacing.
	static auto s_LastTick = std::chrono::steady_clock::now();
	const auto t_Now = std::chrono::steady_clock::now();
	const auto t_Ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_Now - s_LastTick).count();
	const int t_PwmHz = std::max(1, m_HapticConfig.pwmTickHz);
	if (t_Ms < (1000 / t_PwmHz))
	{
		return;
	}
	s_LastTick = t_Now;

	if (!m_HapticHandler.IsEnabled())
	{
		return;
	}

	if (!m_LeftGloveIsHaptics && !m_RightGloveIsHaptics)
	{
		return;
	}

	float t_LeftPowers[NUM_FINGERS_ON_HAND] = {0};
	float t_RightPowers[NUM_FINGERS_ON_HAND] = {0};
	const bool t_LeftActive = m_HapticHandler.ComputeHandPowers(Side::Side_Left, t_LeftPowers);
	const bool t_RightActive = m_HapticHandler.ComputeHandPowers(Side::Side_Right, t_RightPowers);

	const float t_Zero[NUM_FINGERS_ON_HAND] = {0};

	if (m_RightGloveIsHaptics && m_RightHapticGloveID != 0)
	{
		if (t_RightActive)
		{
			VibrateGloveUnchecked(m_RightHapticGloveID, t_RightPowers);
		}
		else if (m_RightHandWasHapticallyActive)
		{
			const int t_ZeroBurst = std::max(1, m_HapticConfig.zeroBurstOnAllOff);
			for (int i = 0; i < t_ZeroBurst; ++i)
			{
				VibrateGloveUnchecked(m_RightHapticGloveID, t_Zero);
			}
		}
	}
	m_RightHandWasHapticallyActive = t_RightActive;

	if (m_LeftGloveIsHaptics && m_LeftHapticGloveID != 0)
	{
		if (t_LeftActive)
		{
			VibrateGloveUnchecked(m_LeftHapticGloveID, t_LeftPowers);
		}
		else if (m_LeftHandWasHapticallyActive)
		{
			const int t_ZeroBurst = std::max(1, m_HapticConfig.zeroBurstOnAllOff);
			for (int i = 0; i < t_ZeroBurst; ++i)
			{
				VibrateGloveUnchecked(m_LeftHapticGloveID, t_Zero);
			}
		}
	}
	m_LeftHandWasHapticallyActive = t_LeftActive;

	m_HapticHandler.MaybePrintDebugStatus();
}

void SDKClient::ToggleWaveHapticsEnabled()
{
	m_HapticHandler.SetEnabled(!m_HapticHandler.IsEnabled());
}

void SDKClient::ToggleHapticsDebugPrint()
{
	m_HapticHandler.SetDebugPrint(!m_HapticHandler.IsDebugPrint());
}

void SDKClient::EnsureAutoUserAssignment()
{
	if (m_AutoUserAssignmentEnabled)
	{
		return;
	}

	if (IsCoreAutoAssignmentEnabledInConfig())
	{
		m_AutoUserAssignmentEnabled = true;
		ClientLog::info(
			"Manus auto user assignment already enabled in Core settings (skipped SDK call)");
		return;
	}

	const SDKReturnCode t_Rc = CoreSdk_SetAutoUserAssignment(true);
	if (t_Rc != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::warn("SetAutoUserAssignment(true) failed rc={}", (int)t_Rc);
		return;
	}

	m_AutoUserAssignmentEnabled = true;
	ClientLog::info(
		"Manus auto user assignment enabled (required for raw skeleton on both dongle slots)");
}

void SDKClient::MaybeLogHandOutputStatus(bool p_PublishedLeft, bool p_PublishedRight)
{
	m_StatusPublishCount++;

	const auto t_Now = std::chrono::steady_clock::now();
	if (m_LastStatusLogTick.time_since_epoch().count() != 0)
	{
		const auto t_Elapsed =
			std::chrono::duration_cast<std::chrono::milliseconds>(t_Now - m_LastStatusLogTick);
		if (t_Elapsed.count() < 3000)
		{
			return;
		}

		const float t_Seconds = static_cast<float>(t_Elapsed.count()) / 1000.0f;
		if (t_Seconds > 0.0f)
		{
			m_EstimatedPublishHz = static_cast<float>(m_StatusPublishCount) / t_Seconds;
		}
	}
	m_LastStatusLogTick = t_Now;
	m_StatusPublishCount = 0;

	auto t_HandLabel = [](bool p_Publishing, std::size_t p_NodeCount) -> std::string {
		if (p_Publishing && p_NodeCount > 0)
		{
			return "publishing (" + std::to_string(p_NodeCount) + " nodes)";
		}
		if (p_NodeCount > 0)
		{
			return "cache stale";
		}
		return "no data";
	};

	std::size_t t_LeftNodes = 0;
	std::size_t t_RightNodes = 0;
	bool t_LeftFresh = false;
	bool t_RightFresh = false;
	{
		std::lock_guard<std::mutex> t_Lock(m_HandSkeletonCacheMutex);
		t_LeftNodes = m_LeftHandSkeletonCache.nodes.size();
		t_RightNodes = m_RightHandSkeletonCache.nodes.size();
		t_LeftFresh = IsHandCacheFresh(m_LeftHandSkeletonCache);
		t_RightFresh = IsHandCacheFresh(m_RightHandSkeletonCache);
	}

	const std::string t_LeftStatus = t_HandLabel(p_PublishedLeft && t_LeftFresh, t_LeftNodes);
	const std::string t_RightStatus = t_HandLabel(p_PublishedRight && t_RightFresh, t_RightNodes);

	std::string t_PairingNote;
	if (m_Landscape != nullptr)
	{
		for (uint32_t i = 0; i < m_Landscape->gloveDevices.gloveCount; i++)
		{
			if (GloveNeedsDongleRepair(m_Landscape->gloveDevices.gloves[i], m_Landscape))
			{
				t_PairingNote = " | pairing: glove not in dongle slot (auto-repairing)";
				break;
			}
		}
	}

	std::ostringstream t_Hz;
	t_Hz << std::fixed << std::setprecision(0) << m_EstimatedPublishHz;

	ClientLog::print(
		"[Mocap] Left: {} | Right: {} | ZMQ {} Hz{}",
		t_LeftStatus,
		t_RightStatus,
		t_Hz.str(),
		t_PairingNote);
}

void SDKClient::ProcessGlovePairing()
{
	const auto t_Now = std::chrono::steady_clock::now();
	if (m_Landscape == nullptr || m_Landscape->gloveDevices.dongleCount == 0)
	{
		return;
	}
	// Delay first tick as well (previously epoch==0 skipped the guard and unpaired at startup).
	if (m_LastPairingTick.time_since_epoch().count() == 0)
	{
		m_LastPairingTick = t_Now;
		return;
	}
	{
		const auto t_Elapsed = std::chrono::duration_cast<std::chrono::seconds>(t_Now - m_LastPairingTick);
		if (t_Elapsed.count() < 5)
		{
			return;
		}
	}
	m_LastPairingTick = t_Now;

	const Landscape* p_Landscape = m_Landscape;

	std::vector<const GloveLandscapeData*> t_GlovesToRepair;
	t_GlovesToRepair.reserve(p_Landscape->gloveDevices.gloveCount);
	for (uint32_t i = 0; i < p_Landscape->gloveDevices.gloveCount; i++)
	{
		const GloveLandscapeData& t_Glove = p_Landscape->gloveDevices.gloves[i];
		if (!GloveNeedsDongleRepair(t_Glove, p_Landscape))
		{
			continue;
		}

		auto t_LastAttempt = m_LastPairAttemptByGlove.find(t_Glove.id);
		if (t_LastAttempt != m_LastPairAttemptByGlove.end())
		{
			const auto t_Elapsed = std::chrono::duration_cast<std::chrono::seconds>(t_Now - t_LastAttempt->second);
			if (t_Elapsed.count() < 10)
			{
				continue;
			}
		}

		t_GlovesToRepair.push_back(&t_Glove);
	}

	if (t_GlovesToRepair.empty())
	{
		return;
	}

	std::sort(
		t_GlovesToRepair.begin(),
		t_GlovesToRepair.end(),
		[](const GloveLandscapeData* a, const GloveLandscapeData* b) {
			if (a->side == b->side)
			{
				return a->id < b->id;
			}
			return static_cast<int>(a->side) < static_cast<int>(b->side);
		});

	for (const GloveLandscapeData* t_GlovePtr : t_GlovesToRepair)
	{
		const GloveLandscapeData& t_Glove = *t_GlovePtr;
		const uint32_t t_TargetDongle = FindTargetDongleForGlove(p_Landscape, t_Glove);
		if (t_TargetDongle == 0)
		{
			continue;
		}

		bool t_UnpairResult = false;
		CoreSdk_UnpairGlove(t_Glove.id, &t_UnpairResult);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		bool t_PairResult = false;
		CoreSdk_PairGloveToDongle(t_Glove.id, t_TargetDongle, &t_PairResult);
		if (!t_PairResult)
		{
			CoreSdk_PairGlove(t_Glove.id, &t_PairResult);
		}
		if (!t_PairResult)
		{
			CoreSdk_PairGloveToDongle(t_Glove.id, t_TargetDongle, &t_PairResult);
		}

		m_LastPairAttemptByGlove[t_Glove.id] = t_Now;
		if (t_PairResult)
		{
			ClientLog::print(
				"[Pairing] Glove 0x{} paired to dongle 0x{}",
				t_Glove.id,
				t_TargetDongle);
		}
	}
}

bool SDKClient::IsKnownDongleId(const Landscape* p_Landscape, uint32_t p_DongleId)
{
	if (p_Landscape == nullptr || p_DongleId == 0)
	{
		return false;
	}

	for (uint32_t d = 0; d < p_Landscape->gloveDevices.dongleCount; d++)
	{
		if (p_Landscape->gloveDevices.dongles[d].id == p_DongleId)
		{
			return true;
		}
	}
	return false;
}

bool SDKClient::IsGloveInDongleSlot(const Landscape* p_Landscape, uint32_t p_GloveId)
{
	if (p_Landscape == nullptr || p_GloveId == 0)
	{
		return false;
	}

	for (uint32_t d = 0; d < p_Landscape->gloveDevices.dongleCount; d++)
	{
		const DongleLandscapeData& t_Dongle = p_Landscape->gloveDevices.dongles[d];
		if (t_Dongle.leftGloveID == p_GloveId || t_Dongle.rightGloveID == p_GloveId)
		{
			return true;
		}
	}
	return false;
}

bool SDKClient::GloveNeedsDongleRepair(const GloveLandscapeData& p_Glove, const Landscape* p_Landscape)
{
	if (p_Landscape == nullptr)
	{
		return false;
	}

	// Core already considers the glove paired / pairing — never Unpair it.
	// Metaglove Pro Haptics often streams fine while landscape slot fields stay empty
	// and may report dongleID == glove.id (seen as "Dongle 0xGLOVE - Glove 0xGLOVE").
	// The previous checks treated that as broken and hammered Unpair/Pair → corrupt
	// skeleton → CasADi nlp_f Inf. Old V1.3 client had no auto-repair at all.
	if (p_Glove.pairedState == DevicePairedState_Paired
		|| p_Glove.pairedState == DevicePairedState_Pairing)
	{
		return false;
	}

	if (IsGloveInDongleSlot(p_Landscape, p_Glove.id))
	{
		return false;
	}

	if (p_Glove.dongleID != 0 && p_Glove.dongleID == p_Glove.id)
	{
		return false;
	}

	if (p_Glove.dongleID != 0 && IsKnownDongleId(p_Landscape, p_Glove.dongleID))
	{
		return false;
	}

	// Only repair when clearly unpaired / no dongle association.
	return p_Glove.dongleID == 0
		|| p_Glove.pairedState == DevicePairedState_Unpaired;
}

uint32_t SDKClient::FindTargetDongleForGlove(const Landscape* p_Landscape, const GloveLandscapeData& p_Glove)
{
	if (p_Landscape == nullptr || p_Landscape->gloveDevices.dongleCount == 0)
	{
		return 0;
	}

	for (uint32_t d = 0; d < p_Landscape->gloveDevices.dongleCount; d++)
	{
		const DongleLandscapeData& t_Dongle = p_Landscape->gloveDevices.dongles[d];
		if (p_Glove.side == Side::Side_Left && t_Dongle.leftGloveID == 0)
		{
			return t_Dongle.id;
		}
		if (p_Glove.side == Side::Side_Right && t_Dongle.rightGloveID == 0)
		{
			return t_Dongle.id;
		}
	}

	return p_Landscape->gloveDevices.dongles[0].id;
}

Side SDKClient::ResolveGloveSide(uint32_t p_GloveId)
{
	if (s_Instance == nullptr || p_GloveId == 0)
	{
		return Side::Side_Invalid;
	}

	// Authoritative: landscape glove device side for this gloveId.
	if (s_Instance->m_Landscape != nullptr)
	{
		for (size_t i = 0; i < s_Instance->m_Landscape->gloveDevices.gloveCount; i++)
		{
			const GloveLandscapeData& t_Glove = s_Instance->m_Landscape->gloveDevices.gloves[i];
			if (t_Glove.id == p_GloveId)
			{
				if (t_Glove.side == Side::Side_Left || t_Glove.side == Side::Side_Right)
				{
					return t_Glove.side;
				}
			}
		}
	}

	// Fallback: user pairing / cached IDs.
	if (s_Instance->m_FirstLeftGloveID != UINT32_MAX && p_GloveId == s_Instance->m_FirstLeftGloveID)
	{
		return Side::Side_Left;
	}
	if (s_Instance->m_FirstRightGloveID != UINT32_MAX && p_GloveId == s_Instance->m_FirstRightGloveID)
	{
		return Side::Side_Right;
	}

	return Side::Side_Invalid;
}

void SDKClient::OnLandscapeCallback(const Landscape* const p_Landscape)
{
	if (s_Instance == nullptr)return;
	Landscape* t_Landscape = new Landscape(*p_Landscape);
	s_Instance->m_Landscape = t_Landscape;

	s_Instance->m_LeftHapticGloveID = 0;
	s_Instance->m_RightHapticGloveID = 0;
	s_Instance->m_LeftGloveIsHaptics = false;
	s_Instance->m_RightGloveIsHaptics = false;
	s_Instance->m_FirstLeftGloveID = UINT32_MAX;
	s_Instance->m_FirstRightGloveID = UINT32_MAX;

	for (size_t i = 0; i < s_Instance->m_Landscape->gloveDevices.gloveCount; i++)
	{
		const GloveLandscapeData& t_Glove = s_Instance->m_Landscape->gloveDevices.gloves[i];

		if (t_Glove.side == Side::Side_Left)
		{
			s_Instance->m_FirstLeftGloveID = t_Glove.id;
			if (t_Glove.isHaptics)
			{
				s_Instance->m_LeftGloveIsHaptics = true;
				s_Instance->m_LeftHapticGloveID = t_Glove.id;
			}
		}
		if (t_Glove.side == Side::Side_Right)
		{
			s_Instance->m_FirstRightGloveID = t_Glove.id;
			if (t_Glove.isHaptics)
			{
				s_Instance->m_RightGloveIsHaptics = true;
				s_Instance->m_RightHapticGloveID = t_Glove.id;
			}
		}
	}
}

ClientReturnCode SDKClient::RegisterAllCallbacks()
{
	// Register the callback to receive Raw Skeleton data
	// it is optional, but without it you can not see any resulting skeleton data.
	// see OnRawSkeletonStreamCallback for more details.
	const SDKReturnCode t_RegisterSkeletonCallbackResult = CoreSdk_RegisterCallbackForRawSkeletonStream(*OnRawSkeletonStreamCallback);
	if (t_RegisterSkeletonCallbackResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to register callback function for processing skeletal data from Manus Core. The value returned was {}.", (int32_t)t_RegisterSkeletonCallbackResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	// Register the callback to receive Landscape data
	const SDKReturnCode t_RegisterLandscapeCallbackResult = CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback);
	if (t_RegisterLandscapeCallbackResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to register callback for landscape from Manus Core. The value returned was {}.", (int32_t)t_RegisterLandscapeCallbackResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKClient::InitializeSDK()
{
	// Use Integrated mode directly, no user input required
	m_ConnectionType = ConnectionType::ConnectionType_Integrated;

	// before we can use the SDK, some internal SDK bits need to be initialized.
	SDKReturnCode t_InitializeResult;
	if (m_ConnectionType == ConnectionType::ConnectionType_Integrated)
	{
		t_InitializeResult = CoreSdk_InitializeIntegrated();
	}
	else
	{
		t_InitializeResult = CoreSdk_InitializeCore();
	}

	if (t_InitializeResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to initialize the Manus Core SDK. The value returned was {}.", (int32_t)t_InitializeResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	const ClientReturnCode t_CallBackResults = RegisterAllCallbacks();
	if (t_CallBackResults != ::ClientReturnCode::ClientReturnCode_Success)
	{
		ClientLog::error("Failed to initialize callbacks.");
		return t_CallBackResults;
	}

	CoordinateSystemVUH t_VUH;
	CoordinateSystemVUH_Init(&t_VUH);
	t_VUH.handedness = Side::Side_Right;
	t_VUH.up = AxisPolarity::AxisPolarity_PositiveZ;
	t_VUH.view = AxisView::AxisView_XFromViewer;
	t_VUH.unitScale = 1.0f;

	const SDKReturnCode t_CoordinateResult = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, true);

	if (t_CoordinateResult != SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::error("Failed to initialize the Manus Core SDK coordinate system. The value returned was {}.", (int32_t)t_CoordinateResult);
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

void SDKClient::LoadGloveCalibration(uint32_t p_GloveId, const std::string& p_CalibrationFileName)
{
	// Get current working directory
	std::string t_CurrentDirectory = std::filesystem::current_path().string();
	
	// Build full path to calibration file
	std::string t_CalibrationFilePath = t_CurrentDirectory + s_SlashForFilesystemPath + p_CalibrationFileName;
	
	// Check if file exists
	if (!DoesFolderOrFileExist(t_CalibrationFilePath))
	{
		ClientLog::warn("Calibration file does not exist: {}", t_CalibrationFilePath);
		return;
	}
	
	// Read file
	std::ifstream t_File = GetInputFileStream(t_CalibrationFilePath);
	if (!t_File)
	{
		ClientLog::warn("Unable to open calibration file: {}", t_CalibrationFilePath);
		return;
	}
	
	// Get file size
	t_File.seekg(0, t_File.end);
	int t_FileLength = (int)t_File.tellg();
	t_File.seekg(0, t_File.beg);
	
	if (t_FileLength <= 0)
	{
		ClientLog::warn("Calibration file is empty: {}", t_CalibrationFilePath);
		t_File.close();
		return;
	}
	
	// Read calibration data
	unsigned char* t_CalibrationData = new unsigned char[t_FileLength];
	t_File.read((char*)t_CalibrationData, t_FileLength);
	t_File.close();
	
	// Set glove calibration
	SetGloveCalibrationReturnCode t_Result;
	SDKReturnCode t_SetResult = CoreSdk_SetGloveCalibration(p_GloveId, t_CalibrationData, t_FileLength, &t_Result);
	
	// Clean up memory
	delete[] t_CalibrationData;
	
	// Print result
	if (t_SetResult == SDKReturnCode::SDKReturnCode_Success)
	{
		ClientLog::print(
			"[Calibration] Loaded {} (glove 0x{})",
			p_CalibrationFileName,
			p_GloveId);
	}
	else
	{
		ClientLog::error("Failed to load glove calibration file: {} (Glove ID: {}), SDK return code: {}", 
			p_CalibrationFileName, p_GloveId, (int)t_SetResult);
	}
}



int main(int argc, char* argv[])
{
	ManusSDK::ClientLog::print("Starting SDK client!");
	
	ClientReturnCode t_Result;
	SDKClient t_SDKClient;

	t_Result = t_SDKClient.Initialize();

	if (t_Result != ClientReturnCode::ClientReturnCode_Success)
	{
		t_SDKClient.ShutDown();
		return static_cast<int>(t_Result);
	}
	ManusSDK::ClientLog::print("SDK client is initialized.");
	ManusSDK::ClientLog::print(
		"Keys: H=manual pulse | T=toggle WAVE haptics | D=haptic debug | J=synth test | Space=exit");

	t_Result = t_SDKClient.Run();
	if (t_Result != ClientReturnCode::ClientReturnCode_Success)
	{
		t_SDKClient.ShutDown();
		return static_cast<int>(t_Result);
	}

	ManusSDK::ClientLog::print("SDK client is done, shutting down.");
	t_Result = t_SDKClient.ShutDown();
	return static_cast<int>(t_Result);
}