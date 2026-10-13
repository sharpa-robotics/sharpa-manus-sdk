#ifndef _SDK_CLIENT_HPP_
#define _SDK_CLIENT_HPP_ 

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <fstream>
#include <memory>
#include <sstream>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <zmq.hpp>

#include "ClientPlatformSpecific.hpp"
#include "HapticConfig.hpp"
#include "HapticFeedbackHandler.hpp"
#include "ManusSDK.h"

// Manus SDK rate limit; PWM tick runs at HAPTIC_PWM_TICK_HZ (can skip sends if too fast).
constexpr unsigned long long MINIMUM_MILLISECONDS_BETWEEN_HAPTICS_COMMANDS = 20;
constexpr int HAPTIC_PWM_TICK_HZ = 50;

// Add ConnectionType enum
enum class ConnectionType : int
{
	ConnectionType_Integrated = 0,
	ConnectionType_Local,
	ConnectionType_Network
};

enum class ClientReturnCode : int
{
	ClientReturnCode_Success = 0,
	ClientReturnCode_FailedPlatformSpecificInitialization,
	ClientReturnCode_FailedToResizeWindow,
	ClientReturnCode_FailedToInitialize,
	ClientReturnCode_FailedToFindHosts,
	ClientReturnCode_FailedToConnect,
	ClientReturnCode_UnrecognizedStateEncountered,
	ClientReturnCode_FailedToShutDownSDK,
	ClientReturnCode_FailedPlatformSpecificShutdown,
	ClientReturnCode_FailedToRestart,
	ClientReturnCode_FailedWrongTimeToGetData,
	ClientReturnCode_MAX_CLIENT_RETURN_CODE_SIZE
};

struct CachedHandSkeleton
{
	std::vector<SkeletonNode> nodes;
	std::chrono::steady_clock::time_point updatedAt{};
	bool valid = false;
};

class SDKClient : public SDKClientPlatformSpecific
{
public:
	SDKClient();
	~SDKClient();

	ClientReturnCode Initialize();
	ClientReturnCode Run();
	ClientReturnCode ShutDown();

	static void OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_Skeleton);
	static void OnLandscapeCallback(const Landscape* const p_Landscape);
	static Side ResolveGloveSide(uint32_t p_GloveId);

	void LoadGloveCalibration(uint32_t p_GloveId, const std::string& p_CalibrationFileName);

	void VibrateGlove(uint32_t p_GloveId, const float p_Powers[NUM_FINGERS_ON_HAND]);
	void StopAllHaptics();
	void RunHapticsTest();
	void RunSyntheticHapticTierTest();

	// WAVE tactile -> Manus haptics (ZMQ SUB on m_HapticSubAddress, proto HapticFeedback).
	void StartHapticFeedbackThread();
	void StopHapticFeedbackThread();
	void HapticFeedbackThreadFunc();
	void ProcessWaveHapticPowers();
	void ToggleWaveHapticsEnabled();
	void ToggleHapticsDebugPrint();
	void ProcessGlovePairing();
	void EnsureAutoUserAssignment();
	void MaybeLogHandOutputStatus(bool p_PublishedLeft, bool p_PublishedRight);
	static bool IsKnownDongleId(const Landscape* p_Landscape, uint32_t p_DongleId);
	static bool IsGloveInDongleSlot(const Landscape* p_Landscape, uint32_t p_GloveId);
	static bool GloveNeedsDongleRepair(const GloveLandscapeData& p_Glove, const Landscape* p_Landscape);
	static uint32_t FindTargetDongleForGlove(const Landscape* p_Landscape, const GloveLandscapeData& p_Glove);
	static Side ResolveSkeletonSideForGlove(uint32_t p_GloveId);
	static bool IsHandCacheFresh(const CachedHandSkeleton& p_Cache);
	void UpdateHandSkeletonCache(Side p_Side, std::vector<SkeletonNode> p_Nodes);

protected:
	bool VibrateGloveUnchecked(uint32_t p_GloveId, const float p_Powers[NUM_FINGERS_ON_HAND]);

	virtual ClientReturnCode InitializeSDK();
	virtual ClientReturnCode RegisterAllCallbacks();
	ClientReturnCode Connect();

protected:
	static SDKClient* s_Instance;
	uint32_t m_FrameId = 0;
	Landscape* m_Landscape = nullptr;
	uint32_t m_FirstLeftGloveID = UINT32_MAX;  // Initialize with invalid value
	uint32_t m_FirstRightGloveID = UINT32_MAX;  // Initialize with invalid value
	uint32_t m_LeftHapticGloveID = 0;
	uint32_t m_RightHapticGloveID = 0;
	bool m_LeftGloveIsHaptics = false;
	bool m_RightGloveIsHaptics = false;
	std::chrono::high_resolution_clock::time_point m_LastLeftHapticsCommandTime{};
	std::chrono::high_resolution_clock::time_point m_LastRightHapticsCommandTime{};

	std::shared_ptr<zmq::context_t> m_ZmqPubContext;
	std::shared_ptr<zmq::socket_t> m_ZmqPublisher;

	// Haptic feedback from retargeting / WAVE (tcp://127.0.0.1:2045).
	std::shared_ptr<zmq::context_t> m_ZmqHapticContext;
	std::shared_ptr<zmq::socket_t> m_ZmqHapticSubscriber;
	std::string m_HapticSubAddress = "tcp://127.0.0.1:2045";
	std::thread m_HapticFeedbackThread;
	std::atomic<bool> m_HapticThreadRunning{false};
	HapticConfig m_HapticConfig;
	HapticFeedbackHandler m_HapticHandler;

	uint32_t m_NumberOfHostsFound = 0;
	uint32_t m_SecondsToFindHosts = 2;
	std::string m_ZmqHost = "tcp://127.0.0.1:2044";  // Bind to localhost port 2044

	std::unique_ptr<ManusHost[]> m_AvailableHosts = nullptr;

	ConnectionType m_ConnectionType = ConnectionType::ConnectionType_Integrated;
	bool m_Running = true;
	
	bool m_LeftGloveCalibrationLoaded = false;
	bool m_RightGloveCalibrationLoaded = false;
	std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> m_LastPairAttemptByGlove;
	CachedHandSkeleton m_LeftHandSkeletonCache;
	CachedHandSkeleton m_RightHandSkeletonCache;
	std::mutex m_HandSkeletonCacheMutex;
	static constexpr int k_HandSkeletonCacheMaxAgeMs = 2000;
	std::chrono::steady_clock::time_point m_LastPairingTick{};
	std::chrono::steady_clock::time_point m_LastStatusLogTick{};
	uint32_t m_StatusPublishCount = 0;
	float m_EstimatedPublishHz = 0.0f;
	bool m_AutoUserAssignmentEnabled = false;
	bool m_LeftHandWasHapticallyActive = false;
	bool m_RightHandWasHapticallyActive = false;
};

#endif
