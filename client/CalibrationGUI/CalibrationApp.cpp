#include "CalibrationApp.hpp"
#include "ClientLogging.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <limits.h>

// GLFW和OpenGL
#define GL_SILENCE_DEPRECATION
#include <GL/gl.h>
#include <GLFW/glfw3.h>

// ImGui
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using namespace ManusSDK;

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
}

CalibrationApp* CalibrationApp::s_Instance = nullptr;

CalibrationApp::CalibrationApp()
{
    s_Instance = this;
    ErgonomicsData_Init(&m_LeftGloveErgoData);
    ErgonomicsData_Init(&m_RightGloveErgoData);
    m_CalibrationCountdown = -1;
    m_ShouldStopCalibration = false;
    GloveCalibrationStepData_Init(&m_StepData);
}

CalibrationApp::~CalibrationApp()
{
    Shutdown();
    s_Instance = nullptr;
}

bool CalibrationApp::Initialize()
{
    // 初始化GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // 创建窗口
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    m_Window = glfwCreateWindow(1800, 1200, "Manus Glove Calibration", nullptr, nullptr);
    if (!m_Window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_Window);
    glfwSwapInterval(1); // 启用垂直同步

    // 初始化ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    
    // 增大字体（增大一倍）
    io.FontGlobalScale = 2.0f;

    // 设置ImGui样式
    ImGui::StyleColorsDark();

    // 初始化ImGui平台/渲染器绑定
    ImGui_ImplGlfw_InitForOpenGL(m_Window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // 初始化SDK
    if (!InitializeSDK())
    {
        return false;
    }

    // 连接Core
    if (!ConnectToCore())
    {
        return false;
    }

    return true;
}

bool CalibrationApp::InitializeSDK()
{
    // 使用Integrated模式（不需要Core运行）
    SDKReturnCode t_InitializeResult = CoreSdk_InitializeIntegrated();
    if (t_InitializeResult != SDKReturnCode_Success)
    {
        ClientLog::error("Failed to initialize SDK");
        return false;
    }
    
    // 设置坐标系
    CoordinateSystemVUH t_VUH;
    CoordinateSystemVUH_Init(&t_VUH);
    t_VUH.handedness = Side::Side_Right;
    t_VUH.up = AxisPolarity::AxisPolarity_PositiveZ;
    t_VUH.view = AxisView::AxisView_XFromViewer;
    t_VUH.unitScale = 1.0f;
    
    SDKReturnCode t_CoordinateResult = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, true);
    if (t_CoordinateResult != SDKReturnCode_Success)
    {
        ClientLog::error("Failed to initialize coordinate system");
        return false;
    }
    
    // 注册回调
    CoreSdk_RegisterCallbackForOnConnect(*OnConnectedCallback);
    CoreSdk_RegisterCallbackForOnDisconnect(*OnDisconnectedCallback);
    CoreSdk_RegisterCallbackForOnLog(*OnLogCallback);
    CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback);
    CoreSdk_RegisterCallbackForErgonomicsStream(*OnErgonomicsCallback);
    CoreSdk_RegisterCallbackForSystemStream(*OnSystemCallback);

    return true;
}

bool CalibrationApp::ConnectToCore()
{
    // Integrated模式需要调用ConnectToHost来建立连接（即使传入空的ManusHost）
    ManusHost t_Empty;
    ManusHost_Init(&t_Empty);
    
    SDKReturnCode t_ConnectResult = CoreSdk_ConnectToHost(t_Empty);
    if (t_ConnectResult != SDKReturnCode_Success)
    {
        ClientLog::error("Failed to connect to Core. Error: {}", (int32_t)t_ConnectResult);
        return false;
    }
    
    ClientLog::print("Connected to integrated Core");
    EnsureAutoUserAssignment();
    return true;
}

void CalibrationApp::Run()
{
    while (!glfwWindowShouldClose(m_Window))
    {
        glfwPollEvents();

        // 开始ImGui帧
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 处理键盘输入
        HandleKeyboardInput();

        // 更新数据
        UpdateData();

        // 渲染GUI
        RenderGUI();

        // 渲染ImGui
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(m_Window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_Window);
    }
}

void CalibrationApp::Shutdown()
{
    if (m_CalibrationThread.joinable())
    {
        m_CalibrationThread.join();
    }

    // Shutdown SDK (this will trigger OnDisconnectedCallback)
    if (m_IsConnected)
    {
        CoreSdk_ShutDown();
        // Give callback a moment to execute
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (m_Window)
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(m_Window);
        glfwTerminate();
        m_Window = nullptr;
    }
}

void CalibrationApp::UpdateData()
{
    // 更新Landscape数据
    m_LandscapeMutex.lock();
    if (m_NewLandscape != nullptr)
    {
        ClientLog::print("[DEBUG] Updating Landscape data...");
        if (m_Landscape != nullptr)
        {
            ClientLog::print("[DEBUG] Deleting old Landscape");
            delete m_Landscape;
        }
        m_Landscape = m_NewLandscape;
        m_NewLandscape = nullptr;
        UpdateResolvedGloveIds();
    }
    m_LandscapeMutex.unlock();

    EnsureAutoUserAssignment();
    ProcessGlovePairing();

    // 更新标定数据
    HandleCalibration();
}

void CalibrationApp::UpdateResolvedGloveIds()
{
    m_FirstLeftGloveID = 0;
    m_FirstRightGloveID = 0;
    if (m_Landscape == nullptr)
    {
        return;
    }

    for (uint32_t i = 0; i < m_Landscape->gloveDevices.gloveCount; i++)
    {
        const GloveLandscapeData& t_Glove = m_Landscape->gloveDevices.gloves[i];
        if (t_Glove.side == Side::Side_Left)
        {
            m_FirstLeftGloveID = t_Glove.id;
        }
        if (t_Glove.side == Side::Side_Right)
        {
            m_FirstRightGloveID = t_Glove.id;
        }
    }
}

void CalibrationApp::EnsureAutoUserAssignment()
{
    if (m_AutoUserAssignmentEnabled)
    {
        return;
    }

    if (IsCoreAutoAssignmentEnabledInConfig())
    {
        m_AutoUserAssignmentEnabled = true;
        ClientLog::print("Manus auto user assignment already enabled in Core settings");
        return;
    }

    const SDKReturnCode t_Rc = CoreSdk_SetAutoUserAssignment(true);
    if (t_Rc != SDKReturnCode_Success)
    {
        ClientLog::warn("SetAutoUserAssignment(true) failed rc={}", (int)t_Rc);
        return;
    }

    m_AutoUserAssignmentEnabled = true;
    ClientLog::print("Manus auto user assignment enabled");
}

bool CalibrationApp::IsKnownDongleId(const Landscape* p_Landscape, uint32_t p_DongleId)
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

bool CalibrationApp::IsGloveInDongleSlot(const Landscape* p_Landscape, uint32_t p_GloveId)
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

bool CalibrationApp::GloveNeedsDongleRepair(
    const GloveLandscapeData& p_Glove,
    const Landscape* p_Landscape)
{
    if (p_Landscape == nullptr)
    {
        return false;
    }

    if (!IsGloveInDongleSlot(p_Landscape, p_Glove.id))
    {
        return true;
    }

    if (p_Glove.dongleID == 0 || p_Glove.dongleID == p_Glove.id)
    {
        return true;
    }

    if (!IsKnownDongleId(p_Landscape, p_Glove.dongleID))
    {
        return true;
    }

    return false;
}

uint32_t CalibrationApp::FindTargetDongleForGlove(
    const Landscape* p_Landscape,
    const GloveLandscapeData& p_Glove)
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

void CalibrationApp::ProcessGlovePairing()
{
    const auto t_Now = std::chrono::steady_clock::now();
    if (m_LastPairingTick.time_since_epoch().count() != 0)
    {
        const auto t_Elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(t_Now - m_LastPairingTick);
        if (t_Elapsed.count() < 5)
        {
            return;
        }
    }
    m_LastPairingTick = t_Now;

    m_LandscapeMutex.lock();
    if (m_Landscape == nullptr || m_Landscape->gloveDevices.dongleCount == 0)
    {
        m_LandscapeMutex.unlock();
        return;
    }

    const Landscape* t_Landscape = m_Landscape;
    std::vector<const GloveLandscapeData*> t_GlovesToRepair;
    t_GlovesToRepair.reserve(t_Landscape->gloveDevices.gloveCount);
    for (uint32_t i = 0; i < t_Landscape->gloveDevices.gloveCount; i++)
    {
        const GloveLandscapeData& t_Glove = t_Landscape->gloveDevices.gloves[i];
        if (!GloveNeedsDongleRepair(t_Glove, t_Landscape))
        {
            continue;
        }

        auto t_LastAttempt = m_LastPairAttemptByGlove.find(t_Glove.id);
        if (t_LastAttempt != m_LastPairAttemptByGlove.end())
        {
            const auto t_Elapsed =
                std::chrono::duration_cast<std::chrono::seconds>(t_Now - t_LastAttempt->second);
            if (t_Elapsed.count() < 10)
            {
                continue;
            }
        }

        t_GlovesToRepair.push_back(&t_Glove);
    }
    m_LandscapeMutex.unlock();

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
        const uint32_t t_TargetDongle = FindTargetDongleForGlove(t_Landscape, t_Glove);
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

void CalibrationApp::RenderGUI()
{
    // 设置主窗口
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("Calibration", nullptr, 
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    // 渲染顶部状态栏
    RenderStatusBar();

    ImGui::Separator();

    // 主内容区域
    ImGui::BeginChild("MainContent", ImVec2(0, -100), false);

    // 左侧：标定信息
    ImGui::BeginChild("CalibrationInfo", ImVec2(ImGui::GetContentRegionAvail().x * 0.7f, 0), false);
    RenderCalibrationInfo();
    ImGui::EndChild();

    ImGui::SameLine();

    // 右侧：按钮面板
    ImGui::BeginChild("ButtonPanel", ImVec2(0, 0), false);
    RenderButtonPanel();
    ImGui::EndChild();

    ImGui::EndChild();

    ImGui::End();
}

void CalibrationApp::RenderStatusBar()
{
    ImGui::Text("Glove Connection Status and Joint Angles");

    ImGui::Columns(2, "StatusColumns", false);

    // Left Glove
    uint32_t leftGloveID = 0;
    m_LandscapeMutex.lock();
    leftGloveID = m_FirstLeftGloveID;
    m_LandscapeMutex.unlock();
    
    ImGui::Text("Left Glove:");
    bool leftConnected = IsGloveConnected(leftGloveID);
    ImGui::SameLine();
    ImGui::TextColored(leftConnected ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1), 
        leftConnected ? "Connected" : "Disconnected");
    
    if (leftConnected && leftGloveID != 0)
    {
        ImGui::Text("ID: 0x%X", leftGloveID);
        ImGui::Text("Joint Angles:");
        m_ErgoMutex.lock();
        if (m_LeftGloveErgoData.id == leftGloveID)
        {
            ImGui::TextWrapped("%s", GetJointAnglesString(m_LeftGloveErgoData, "left").c_str());
        }
        else
        {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Waiting for data...");
        }
        m_ErgoMutex.unlock();
    }

    ImGui::NextColumn();

    // Right Glove
    uint32_t rightGloveID = 0;
    m_LandscapeMutex.lock();
    rightGloveID = m_FirstRightGloveID;
    m_LandscapeMutex.unlock();
    
    ImGui::Text("Right Glove:");
    bool rightConnected = IsGloveConnected(rightGloveID);
    ImGui::SameLine();
    ImGui::TextColored(rightConnected ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1), 
        rightConnected ? "Connected" : "Disconnected");
    
    if (rightConnected && rightGloveID != 0)
    {
        ImGui::Text("ID: 0x%X", rightGloveID);
        ImGui::Text("Joint Angles:");
        m_ErgoMutex.lock();
        if (m_RightGloveErgoData.id == rightGloveID)
        {
            ImGui::TextWrapped("%s", GetJointAnglesString(m_RightGloveErgoData, "right").c_str());
        }
        else
        {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Waiting for data...");
        }
        m_ErgoMutex.unlock();
    }

    ImGui::Columns(1);
}

void CalibrationApp::RenderButtonPanel()
{
    ImGui::Text("Control Panel");
    ImGui::Separator();

    // Left Glove按钮
    if (ImGui::Button("Left Glove", ImVec2(-1, 0)))
    {
        ClientLog::print("[DEBUG] Left Glove button clicked");
        std::ostringstream debug_oss_btn1;
        debug_oss_btn1 << "[DEBUG] Current state: m_IsCalibrationInProgress=" << m_IsCalibrationInProgress 
                       << ", m_CalibrationGloveId=0x" << std::hex << m_CalibrationGloveId << std::dec;
        ClientLog::print(debug_oss_btn1.str().c_str());
        
        // 如果正在标定，先停止
        if (m_IsCalibrationInProgress)
        {
            ClientLog::print("[DEBUG] Waiting for calibration thread to finish...");
            if (m_CalibrationThread.joinable())
            {
                m_CalibrationThread.join();
                ClientLog::print("[DEBUG] Calibration thread joined");
            }
        }
        
        ClientLog::print("[DEBUG] Setting m_CalibrateLeftHand = true");
        m_CalibrateLeftHand = true;
        ClientLog::print("[DEBUG] Calling RestartCalibration()");
        RestartCalibration();
        ClientLog::print("[DEBUG] RestartCalibration() returned");
    }

    // Right Glove按钮
    if (ImGui::Button("Right Glove", ImVec2(-1, 0)))
    {
        ClientLog::print("[DEBUG] Right Glove button clicked");
        std::ostringstream debug_oss_btn2;
        debug_oss_btn2 << "[DEBUG] Current state: m_IsCalibrationInProgress=" << m_IsCalibrationInProgress 
                       << ", m_CalibrationGloveId=0x" << std::hex << m_CalibrationGloveId << std::dec;
        ClientLog::print(debug_oss_btn2.str().c_str());
        
        // 如果正在标定，先停止
        if (m_IsCalibrationInProgress)
        {
            ClientLog::print("[DEBUG] Waiting for calibration thread to finish...");
            if (m_CalibrationThread.joinable())
            {
                m_CalibrationThread.join();
                ClientLog::print("[DEBUG] Calibration thread joined");
            }
        }
        
        ClientLog::print("[DEBUG] Setting m_CalibrateLeftHand = false");
        m_CalibrateLeftHand = false;
        ClientLog::print("[DEBUG] Calling RestartCalibration()");
        RestartCalibration();
        ClientLog::print("[DEBUG] RestartCalibration() returned");
    }

    ImGui::Separator();

    // Start Calibration按钮 (F5)
    bool canStart = !m_IsCalibrationInProgress && m_CalibrationGloveId != 0 && !m_CalibrationStarted;
    if (!canStart)
        ImGui::BeginDisabled();
    
    if (ImGui::Button("Start Calibration (F5)", ImVec2(-1, 0)) || 
        (m_F5Pressed && canStart))
    {
        StartCalibration();
        m_F5Pressed = false; // 重置状态
    }
    
    if (!canStart)
        ImGui::EndDisabled();

    // Next Step按钮 (F9)
    bool canNext = m_CalibrationStarted && !m_IsCalibrationInProgress && 
                   m_CalibrationStep < m_NumberOfCalibrationSteps &&
                   m_CalibrationGloveId != 0;
    if (!canNext)
        ImGui::BeginDisabled();
    
    if (ImGui::Button("Next Step (F9)", ImVec2(-1, 0)) || 
        (m_F9Pressed && canNext))
    {
        NextStep();
        m_F9Pressed = false; // 重置状态
    }
    
    if (!canNext)
        ImGui::EndDisabled();

    // Restart Calibration按钮
    if (ImGui::Button("Restart Calibration", ImVec2(-1, 0)))
    {
        RestartCalibration();
    }
}

void CalibrationApp::RenderCalibrationInfo()
{
    ImGui::Text("Calibration Info");
    ImGui::Separator();

    std::string side = m_CalibrateLeftHand ? "Left Glove" : "Right Glove";
    ImGui::Text("Current Calibration: %s", side.c_str());

    if (m_CalibrationGloveId != 0)
    {
        ImGui::Text("Glove ID: 0x%X", m_CalibrationGloveId);
        ImGui::Text("Total Steps: %u", m_NumberOfCalibrationSteps);
        
        // Show hint before starting calibration
        if (!m_CalibrationStarted)
        {
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 0, 1));
            ImGui::TextWrapped("Description: Please place your hand flat with thumb pointing outward, away from the metal surface, then click start");
            ImGui::PopStyleColor();
        }
        else
        {
            ImGui::Text("Current Step: %u / %u", m_CalibrationStep + 1, m_NumberOfCalibrationSteps);
            
            // Show description for all steps
            if (m_StepData.description[0] != '\0' || m_CalibrationStep == 0)
            {
                ImGui::Separator();
                // For step 1 (index 0), show enhanced description
                if (m_CalibrationStep == 0)
                {
                    ImGui::TextWrapped("Description: Please place your hand flat with thumb pointing outward, away from the metal surface");
                }
                else
                {
                    ImGui::TextWrapped("Description: %s", m_StepData.description);
                }
                
                // Show countdown for steps 1 and 2 (index 0 and 1)
                if ((m_CalibrationStep == 0 || m_CalibrationStep == 1) && m_CalibrationCountdown >= 0)
                {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Recording gesture in %d seconds...", m_CalibrationCountdown);
                }
                // Show continuous calculation for steps 3 and 4 (index 2 and 3)
                else if (m_CalibrationStep >= 2)
                {
                    if (m_StepData.time < 0)
                    {
                        ImGui::TextColored(ImVec4(1, 1, 0, 1), "Continuous Calculation Step");
                    }
                    else
                    {
                        ImGui::Text("Time: %.2f seconds", m_StepData.time);
                    }
                }
            }
        }

        if (!m_CalibrationMessage.empty())
        {
            ImGui::Separator();
            ImVec4 color = (m_CalibrationMessage.find("finished") != std::string::npos || 
                           m_CalibrationMessage.find("complete") != std::string::npos ||
                           m_CalibrationMessage.find("success") != std::string::npos) ? 
                           ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1);
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", m_CalibrationMessage.c_str());
            ImGui::PopStyleColor();
        }

        if (m_IsCalibrationInProgress)
        {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Calibration in progress...");
        }
    }
    else
    {
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "No glove selected or glove not connected");
    }
}

void CalibrationApp::HandleCalibration()
{
    std::ostringstream debug_oss;
    debug_oss << "[DEBUG] HandleCalibration() called, m_CalibrateLeftHand=" << m_CalibrateLeftHand;
    ClientLog::print(debug_oss.str().c_str());
    
    // Use mutex to safely access Landscape
    ClientLog::print("[DEBUG] Acquiring Landscape mutex...");
    m_LandscapeMutex.lock();
    ClientLog::print("[DEBUG] Landscape mutex acquired");
    
    // Safety check: ensure Landscape is valid
    if (m_Landscape == nullptr)
    {
        ClientLog::print("[DEBUG] m_Landscape is nullptr, unlocking and returning");
        m_LandscapeMutex.unlock();
        m_CalibrationGloveId = 0;
        return;
    }
    
    std::ostringstream debug_oss2;
    debug_oss2 << "[DEBUG] m_Landscape is valid, gloveCount="
               << m_Landscape->gloveDevices.gloveCount
               << " left=0x" << std::hex << m_FirstLeftGloveID
               << " right=0x" << m_FirstRightGloveID << std::dec;
    ClientLog::print(debug_oss2.str().c_str());

    uint32_t t_GloveID = m_CalibrateLeftHand ? m_FirstLeftGloveID : m_FirstRightGloveID;
    
    std::ostringstream debug_oss3;
    debug_oss3 << "[DEBUG] Got glove ID: 0x" << std::hex << t_GloveID << std::dec 
               << " (hand=" << (m_CalibrateLeftHand ? "left" : "right") << ")";
    ClientLog::print(debug_oss3.str().c_str());
    
    m_LandscapeMutex.unlock();
    ClientLog::print("[DEBUG] Landscape mutex released");

    if (t_GloveID == 0)
    {
        ClientLog::print("[DEBUG] Glove ID is 0, returning");
        m_CalibrationGloveId = 0;
        return;
    }

    // 检查手套是否连接（IsGloveConnected内部会加锁）
    std::ostringstream debug_oss4;
    debug_oss4 << "[DEBUG] Checking if glove 0x" << std::hex << t_GloveID << std::dec << " is connected...";
    ClientLog::print(debug_oss4.str().c_str());
    bool connected = IsGloveConnected(t_GloveID);
    ClientLog::print("[DEBUG] Glove connected: {}", connected);
    
    if (!connected)
    {
        ClientLog::print("[DEBUG] Glove not connected, returning");
        m_CalibrationGloveId = 0;
        return;
    }

    // Only update glove ID if it changed, to avoid interrupting calibration
    if (m_CalibrationGloveId != t_GloveID && m_IsCalibrationInProgress)
    {
        ClientLog::print("[DEBUG] Calibration in progress, not changing glove ID");
        return;
    }

    std::ostringstream debug_oss5;
    debug_oss5 << "[DEBUG] Setting m_CalibrationGloveId = 0x" << std::hex << t_GloveID << std::dec;
    ClientLog::print(debug_oss5.str().c_str());
    m_CalibrationGloveId = t_GloveID;

    // 获取标定步骤数
    GloveCalibrationArgs t_Args;
    t_Args.gloveId = m_CalibrationGloveId;
    
    ClientLog::print("[DEBUG] Calling CoreSdk_GloveCalibrationGetNumberOfSteps...");
    SDKReturnCode t_Res = CoreSdk_GloveCalibrationGetNumberOfSteps(t_Args, &m_NumberOfCalibrationSteps);
    std::ostringstream debug_oss6;
    debug_oss6 << "[DEBUG] CoreSdk_GloveCalibrationGetNumberOfSteps returned: " << (int32_t)t_Res 
               << ", steps=" << m_NumberOfCalibrationSteps;
    ClientLog::print(debug_oss6.str().c_str());
    
    if (t_Res != SDKReturnCode_Success || m_NumberOfCalibrationSteps == 0)
    {
        ClientLog::print("[DEBUG] Failed to get number of steps or steps is 0, returning");
        return;
    }

    // 确保步骤索引有效
    if (m_CalibrationStep >= m_NumberOfCalibrationSteps)
    {
        std::ostringstream debug_oss7;
        debug_oss7 << "[DEBUG] Clamping step from " << m_CalibrationStep << " to " << (m_NumberOfCalibrationSteps - 1);
        ClientLog::print(debug_oss7.str().c_str());
        m_CalibrationStep = m_NumberOfCalibrationSteps - 1;
    }

    // 获取当前步骤数据（只在不在进行中时更新，避免覆盖）
    if (!m_IsCalibrationInProgress)
    {
        std::ostringstream debug_oss8;
        debug_oss8 << "[DEBUG] Getting step data for step " << m_CalibrationStep << "...";
        ClientLog::print(debug_oss8.str().c_str());
        GloveCalibrationStepArgs t_StepArgs;
        t_StepArgs.gloveId = m_CalibrationGloveId;
        t_StepArgs.stepIndex = m_CalibrationStep;
        t_Res = CoreSdk_GloveCalibrationGetStepData(t_StepArgs, &m_StepData);
        std::ostringstream debug_oss9;
        debug_oss9 << "[DEBUG] CoreSdk_GloveCalibrationGetStepData returned: " << (int32_t)t_Res;
        ClientLog::print(debug_oss9.str().c_str());
        
        if (t_Res != SDKReturnCode_Success)
        {
            ClientLog::print("[DEBUG] Failed to get step data, returning");
            return;
        }
        ClientLog::print("[DEBUG] Step data retrieved successfully");
    }
    else
    {
        ClientLog::print("[DEBUG] Calibration in progress, skipping step data update");
    }
    
    ClientLog::print("[DEBUG] HandleCalibration() completed successfully");
}

void CalibrationApp::StartCalibration()
{
    if (m_IsCalibrationInProgress || m_CalibrationGloveId == 0)
        return;

    GloveCalibrationArgs t_Args;
    t_Args.gloveId = m_CalibrationGloveId;

    bool t_Result = false;
    SDKReturnCode t_Res = CoreSdk_GloveCalibrationStart(t_Args, &t_Result);

    if (t_Res == SDKReturnCode_Success && t_Result)
    {
        m_CalibrationStarted = true;
        m_CalibrationStep = 0;
        
        // 获取第一步数据
        GloveCalibrationStepArgs t_StepArgs;
        t_StepArgs.gloveId = m_CalibrationGloveId;
        t_StepArgs.stepIndex = 0;
        SDKReturnCode t_StepRes = CoreSdk_GloveCalibrationGetStepData(t_StepArgs, &m_StepData);
        
        if (t_StepRes == SDKReturnCode_Success)
        {
            // Execute first step 5 second delay to give user time to position hand
            m_CalibrationMessage = "Calibration started - Step 1 in progress...";
            ExecuteCalibrationStep();
        }
        else
        {
            m_CalibrationMessage = "Failed to get first step data";
        }
    }
    else
    {
        m_CalibrationMessage = "Failed to start calibration";
    }
}

void CalibrationApp::NextStep()
{
    ClientLog::print("[DEBUG] NextStep() called");
    std::ostringstream debug_oss17;
    debug_oss17 << "[DEBUG] State: m_CalibrationStarted=" << m_CalibrationStarted 
                << ", m_CalibrationGloveId=0x" << std::hex << m_CalibrationGloveId << std::dec
                << ", m_CalibrationStep=" << m_CalibrationStep << "/" << m_NumberOfCalibrationSteps 
                << ", m_IsCalibrationInProgress=" << m_IsCalibrationInProgress;
    ClientLog::print(debug_oss17.str().c_str());
    
    // 如果标定未开始或手套无效，返回
    if (!m_CalibrationStarted || m_CalibrationGloveId == 0)
    {
        ClientLog::print("[DEBUG] Calibration not started or invalid glove ID, returning");
        return;
    }

    // 如果当前步骤正在执行，等待完成
    if (m_IsCalibrationInProgress)
    {
        ClientLog::print("[DEBUG] Calibration in progress, waiting for thread...");
        if (m_CalibrationThread.joinable())
        {
            m_CalibrationThread.join();
            ClientLog::print("[DEBUG] Thread joined");
        }
    }

    // 检查是否是最后一步
    if (m_CalibrationStep >= m_NumberOfCalibrationSteps - 1)
    {
        ClientLog::print("[DEBUG] Last step reached, finishing calibration...");
        // 如果是最后一步且已完成，则完成标定
        if (!m_IsCalibrationInProgress)
        {
            GloveCalibrationArgs t_Args;
            t_Args.gloveId = m_CalibrationGloveId;
            bool t_Result = false;
            SDKReturnCode t_Res = CoreSdk_GloveCalibrationFinish(t_Args, &t_Result);
            ClientLog::print("[DEBUG] CoreSdk_GloveCalibrationFinish returned: {}, result={}", (int32_t)t_Res, t_Result);
            
            if (t_Res == SDKReturnCode_Success && t_Result)
            {
                m_CalibrationMessage = "Calibration complete!";
                
                // Save calibration result
                std::string handName = m_CalibrateLeftHand ? "left" : "right";
                if (SaveCalibrationToFile(m_CalibrationGloveId, handName))
                {
                    m_CalibrationMessage += "\nSaved to: Calibration_" + handName + ".mcal";
                }
                else
                {
                    m_CalibrationMessage += "\nFailed to save calibration file";
                }
                
                // Reset to initial state after saving
                ClientLog::print("[DEBUG] Resetting to initial state after calibration complete...");
                m_CalibrationStarted = false;
                m_CalibrationStep = 0;
                m_IsCalibrationInProgress = false;
                m_NumberOfCalibrationSteps = 0;
                GloveCalibrationStepData_Init(&m_StepData);
                // Keep m_CalibrationGloveId and m_CalibrateLeftHand so user can see which glove was calibrated
            }
            else
            {
                m_CalibrationMessage = "Failed to finish calibration";
                m_CalibrationStarted = false;
                m_CalibrationStep = 0;
            }
        }
        ClientLog::print("[DEBUG] NextStep() returning (last step)");
        return;
    }

    // 进入下一步（相当于原本代码中的'P'）
    m_CalibrationStep++;
            std::ostringstream debug_oss21;
            debug_oss21 << "[DEBUG] Moving to next step: " << m_CalibrationStep;
            ClientLog::print(debug_oss21.str().c_str());
    
    // 获取下一步数据
    GloveCalibrationStepArgs t_StepArgs;
    t_StepArgs.gloveId = m_CalibrationGloveId;
    t_StepArgs.stepIndex = m_CalibrationStep;
    
    std::ostringstream debug_oss22;
    debug_oss22 << "[DEBUG] Getting step data for step " << m_CalibrationStep << "...";
    ClientLog::print(debug_oss22.str().c_str());
    SDKReturnCode t_Res = CoreSdk_GloveCalibrationGetStepData(t_StepArgs, &m_StepData);
    std::ostringstream debug_oss23;
    debug_oss23 << "[DEBUG] CoreSdk_GloveCalibrationGetStepData returned: " << (int32_t)t_Res;
    ClientLog::print(debug_oss23.str().c_str());
    
    if (t_Res == SDKReturnCode_Success)
    {
        ClientLog::print("[DEBUG] Step data retrieved, executing step...");
        
        // Show appropriate message before executing step
        if (m_CalibrationStep == 1)
        {
            // Step 2: Make a fist
            m_CalibrationMessage = "Step 2 in progress...";
        }
        else
        {
            m_CalibrationMessage = "Step " + std::to_string(m_CalibrationStep + 1) + " in progress...";
        }
        
        // Automatically execute next step
        ExecuteCalibrationStep();
        
        ClientLog::print("[DEBUG] NextStep() completed successfully");
    }
    else
    {
        ClientLog::print("[DEBUG] Failed to get step data");
        m_CalibrationMessage = "Failed to get step data";
    }
}

void CalibrationApp::ExecuteCalibrationStep()
{
    ClientLog::print("[DEBUG] ExecuteCalibrationStep() called");
    std::ostringstream debug_oss18;
    debug_oss18 << "[DEBUG] State: m_IsCalibrationInProgress=" << m_IsCalibrationInProgress 
                << ", m_CalibrationGloveId=0x" << std::hex << m_CalibrationGloveId << std::dec
                << ", m_CalibrationStep=" << m_CalibrationStep;
    ClientLog::print(debug_oss18.str().c_str());
    
    if (m_IsCalibrationInProgress || m_CalibrationGloveId == 0)
    {
        ClientLog::print("[DEBUG] Cannot execute: in progress={}, gloveId=0x{:X}", m_IsCalibrationInProgress, m_CalibrationGloveId);
        return;
    }

    // Ensure previous thread is finished
    if (m_CalibrationThread.joinable())
    {
        ClientLog::print("[DEBUG] Previous thread exists, joining...");
        m_CalibrationThread.join();
        ClientLog::print("[DEBUG] Previous thread joined");
    }

    m_IsCalibrationInProgress = true;
    
    // Capture current step index to avoid race condition
    uint32_t currentStep = m_CalibrationStep;
    uint32_t currentGloveId = m_CalibrationGloveId;
    m_CalibrationMessage = "Step " + std::to_string(currentStep + 1) + " in progress...";
    
    std::ostringstream debug_oss19;
    debug_oss19 << "[DEBUG] Starting calibration step " << currentStep 
                << " for glove 0x" << std::hex << currentGloveId << std::dec;
    ClientLog::print(debug_oss19.str().c_str());

    GloveCalibrationStepArgs t_StepArgs;
    t_StepArgs.gloveId = currentGloveId;
    t_StepArgs.stepIndex = currentStep;

    // Execute in separate thread
    ClientLog::print("[DEBUG] Creating calibration thread...");
    m_CalibrationThread = std::thread([this, t_StepArgs, currentStep, currentGloveId]() {
        ClientLog::print("[DEBUG] Calibration thread started for step {}", currentStep);
        
        // Countdown 5 seconds for step 0 (first step) and step 1 (make fist step) to give user time to position hand
        if (currentStep == 0 || currentStep == 1)
        {
            ClientLog::print("[DEBUG] Starting 5 second countdown before step {} calculation...", currentStep + 1);
            for (int i = 5; i > 0; --i)
            {
                // Check if calibration should be stopped (e.g., user clicked restart)
                if (s_Instance && s_Instance->m_ShouldStopCalibration.load())
                {
                    ClientLog::print("[DEBUG] Calibration stopped during countdown");
                    if (s_Instance)
                    {
                        s_Instance->m_CalibrationCountdown = -1; // Reset countdown
                        s_Instance->m_IsCalibrationInProgress = false;
                    }
                    return; // Exit thread early
                }
                
                if (s_Instance)
                {
                    s_Instance->m_CalibrationCountdown = i;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            if (s_Instance)
            {
                s_Instance->m_CalibrationCountdown = -1; // Reset countdown
            }
            ClientLog::print("[DEBUG] Countdown completed, starting step {} calculation...", currentStep + 1);
        }
        
        // Check again before starting calibration step (in case stop was requested during countdown)
        if (s_Instance && s_Instance->m_ShouldStopCalibration.load())
        {
            ClientLog::print("[DEBUG] Calibration stopped before starting step");
            if (s_Instance)
            {
                s_Instance->m_IsCalibrationInProgress = false;
            }
            return; // Exit thread early
        }
        
        bool t_Result = false;
        SDKReturnCode t_Res = CoreSdk_GloveCalibrationStartStep(t_StepArgs, &t_Result);
        
        std::ostringstream debug_oss20;
        debug_oss20 << "[DEBUG] CoreSdk_GloveCalibrationStartStep returned: " << (int32_t)t_Res 
                    << ", result=" << t_Result;
        ClientLog::print(debug_oss20.str().c_str());

        // Use mutex to safely update shared state
        if (s_Instance)
        {
            s_Instance->m_IsCalibrationInProgress = false;

            if (t_Res == SDKReturnCode_Success && t_Result)
            {
                std::string stepMessage = "Step " + std::to_string(currentStep + 1) + " complete!";
                // After step 1 (index 0) completes, show hint for step 2
                if (currentStep == 0)
                {
                    stepMessage += "\nDescription: Please make a fist, then click next step";
                }
                // After step 2 (index 1) completes, get step 3 description if available
                else if (currentStep == 1)
                {
                    // Get step 3 data to show its description
                    GloveCalibrationStepArgs t_NextStepArgs;
                    t_NextStepArgs.gloveId = currentGloveId;
                    t_NextStepArgs.stepIndex = 2;
                    GloveCalibrationStepData t_NextStepData;
                    GloveCalibrationStepData_Init(&t_NextStepData);
                    SDKReturnCode t_NextRes = CoreSdk_GloveCalibrationGetStepData(t_NextStepArgs, &t_NextStepData);
                    if (t_NextRes == SDKReturnCode_Success && t_NextStepData.description[0] != '\0')
                    {
                        stepMessage += " Click next step";
                    }
                }
                // After step 3 (index 2) completes, get step 4 description if available
                else if (currentStep == 2)
                {
                    // Get step 4 data to show its description
                    GloveCalibrationStepArgs t_NextStepArgs;
                    t_NextStepArgs.gloveId = currentGloveId;
                    t_NextStepArgs.stepIndex = 3;
                    GloveCalibrationStepData t_NextStepData;
                    GloveCalibrationStepData_Init(&t_NextStepData);
                    SDKReturnCode t_NextRes = CoreSdk_GloveCalibrationGetStepData(t_NextStepArgs, &t_NextStepData);
                    if (t_NextRes == SDKReturnCode_Success && t_NextStepData.description[0] != '\0')
                    {
                        stepMessage += " Click next step" ;
                    }
                }
                // After step 4 (index 3) completes, show hint for saving
                else if (currentStep == 3)
                {
                    stepMessage += "\nNext step for save calibration result";
                }
                s_Instance->m_CalibrationMessage = stepMessage;
                std::ostringstream debug_oss26;
        debug_oss26 << "[DEBUG] Step " << (currentStep + 1) << " completed successfully";
        ClientLog::print(debug_oss26.str().c_str());
            }
            else
            {
                s_Instance->m_CalibrationMessage = "Step " + std::to_string(currentStep + 1) + " failed";
                std::ostringstream debug_oss27;
                debug_oss27 << "[DEBUG] Step " << (currentStep + 1) << " failed";
                ClientLog::print(debug_oss27.str().c_str());
            }
        }
        else
        {
            ClientLog::print("[DEBUG] s_Instance is null in thread!");
        }
        
        ClientLog::print("[DEBUG] Calibration thread finishing");
    });
    ClientLog::print("[DEBUG] Calibration thread created");
}

void CalibrationApp::RestartCalibration()
{
    ClientLog::print("[DEBUG] RestartCalibration() called");
    std::ostringstream debug_oss14;
    debug_oss14 << "[DEBUG] Current state: m_IsCalibrationInProgress=" << m_IsCalibrationInProgress 
                << ", m_CalibrationGloveId=0x" << std::hex << m_CalibrationGloveId << std::dec
                << ", m_CalibrationStarted=" << m_CalibrationStarted;
    ClientLog::print(debug_oss14.str().c_str());
    
    // Set flag to stop current calibration (this will be checked by the worker thread)
    m_ShouldStopCalibration = true;
    m_CalibrationCountdown = -1; // Reset countdown immediately
    
    // Wait for any running calibration thread
    if (m_IsCalibrationInProgress)
    {
        ClientLog::print("[DEBUG] Calibration in progress, waiting for thread to stop...");
        if (m_CalibrationThread.joinable())
        {
            ClientLog::print("[DEBUG] Thread is joinable, joining...");
            m_CalibrationThread.join();
            ClientLog::print("[DEBUG] Thread joined successfully");
        }
        else
        {
            ClientLog::print("[DEBUG] Thread is not joinable");
        }
    }
    else if (m_CalibrationThread.joinable())
    {
        ClientLog::print("[DEBUG] Thread exists but not in progress, joining to clean up...");
        m_CalibrationThread.join();
        ClientLog::print("[DEBUG] Thread joined successfully");
    }
    else
    {
        ClientLog::print("[DEBUG] No thread to join");
    }
    
    // Reset the stop flag for next calibration
    m_ShouldStopCalibration = false;

    // Stop calibration if active (use saved glove ID to avoid race condition)
    // Only stop if calibration was actually started
    uint32_t savedGloveId = m_CalibrationGloveId;
    std::ostringstream debug_oss25;
    debug_oss25 << "[DEBUG] Saved glove ID: 0x" << std::hex << savedGloveId << std::dec;
    ClientLog::print(debug_oss25.str().c_str());
    
    if (savedGloveId != 0 && m_CalibrationStarted)
    {
        std::ostringstream debug_oss15;
        debug_oss15 << "[DEBUG] Stopping calibration for glove 0x" << std::hex << savedGloveId << std::dec << "...";
        ClientLog::print(debug_oss15.str().c_str());
        GloveCalibrationArgs t_Args;
        t_Args.gloveId = savedGloveId;
        bool t_Result = false;
        SDKReturnCode t_Res = CoreSdk_GloveCalibrationStop(t_Args, &t_Result);
        std::ostringstream debug_oss16;
        debug_oss16 << "[DEBUG] CoreSdk_GloveCalibrationStop returned: " << (int32_t)t_Res << ", result=" << t_Result;
        ClientLog::print(debug_oss16.str().c_str());
    }
    else
    {
        if (savedGloveId == 0)
        {
            ClientLog::print("[DEBUG] No glove ID to stop calibration");
        }
        else
        {
            ClientLog::print("[DEBUG] Calibration not started, skipping stop");
        }
    }

    // Reset calibration state
    ClientLog::print("[DEBUG] Resetting calibration state...");
    m_CalibrationStarted = false;
    m_CalibrationStep = 0;
    m_IsCalibrationInProgress = false;
    m_CalibrationMessage = "";
    m_NumberOfCalibrationSteps = 0;
    m_CalibrationCountdown = -1;
    m_ShouldStopCalibration = false;
    GloveCalibrationStepData_Init(&m_StepData);
    
    // Reset calibration glove ID - will be set by HandleCalibration on next update
    m_CalibrationGloveId = 0;
    
    ClientLog::print("[DEBUG] RestartCalibration() completed");
}

bool CalibrationApp::IsGloveConnected(uint32_t p_GloveId)
{
    std::ostringstream debug_oss10;
    debug_oss10 << "[DEBUG] IsGloveConnected(0x" << std::hex << p_GloveId << std::dec << ") called";
    ClientLog::print(debug_oss10.str().c_str());
    
    if (p_GloveId == 0)
    {
        ClientLog::print("[DEBUG] Glove ID is 0, returning false");
        return false;
    }
    
    // Use mutex to safely access Landscape
    ClientLog::print("[DEBUG] Acquiring Landscape mutex in IsGloveConnected...");
    m_LandscapeMutex.lock();
    ClientLog::print("[DEBUG] Landscape mutex acquired in IsGloveConnected");
    
    bool connected = false;
    
    if (m_Landscape == nullptr)
    {
        ClientLog::print("[DEBUG] m_Landscape is nullptr");
    }
    else if (m_Landscape->gloveDevices.gloveCount == 0)
    {
        ClientLog::print("[DEBUG] gloveCount is 0");
    }
    else
    {
        std::ostringstream debug_oss11;
        debug_oss11 << "[DEBUG] Checking " << m_Landscape->gloveDevices.gloveCount << " gloves...";
        ClientLog::print(debug_oss11.str().c_str());
        for (size_t i = 0; i < m_Landscape->gloveDevices.gloveCount; i++)
        {
            uint32_t gloveId = m_Landscape->gloveDevices.gloves[i].id;
            std::ostringstream debug_oss12;
            debug_oss12 << "[DEBUG] Glove[" << i << "]: ID=0x" << std::hex << gloveId << std::dec;
            ClientLog::print(debug_oss12.str().c_str());
            if (gloveId == p_GloveId)
            {
                connected = true;
                ClientLog::print("[DEBUG] Found matching glove!");
                break;
            }
        }
    }
    
    m_LandscapeMutex.unlock();
    std::ostringstream debug_oss13;
    debug_oss13 << "[DEBUG] IsGloveConnected returning: " << connected;
    ClientLog::print(debug_oss13.str().c_str());
    return connected;
}

std::string CalibrationApp::GetJointAnglesString(const ErgonomicsData& p_ErgoData, const std::string& p_Hand)
{
    const std::string t_FingerNames[5] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
    const std::string t_FingerJointNames[3] = { "MCP", "PIP", "DIP" };
    const std::string t_ThumbJointNames[3] = { "CMC", "MCP", "IP" };

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    
    // 右手数据偏移量为20（与原始代码一致）
    int t_DataOffset = 0;
    if (p_Hand == "right") t_DataOffset = 20;

    const std::string* t_JointNames = t_ThumbJointNames;
    for (unsigned int t_FingerNumber = 0; t_FingerNumber < 5; t_FingerNumber++)
    {
        // 使用固定宽度格式对齐
        oss << std::setw(6) << std::left << t_FingerNames[t_FingerNumber] << ": ";
        oss << std::setw(4) << t_JointNames[0] << "=" << std::setw(6) << std::right << RoundFloat(p_ErgoData.data[t_DataOffset], 2) << "deg ";
        oss << std::setw(4) << t_JointNames[0] << "s=" << std::setw(6) << std::right << RoundFloat(p_ErgoData.data[t_DataOffset + 1], 2) << "deg ";
        oss << std::setw(4) << t_JointNames[1] << "=" << std::setw(6) << std::right << RoundFloat(p_ErgoData.data[t_DataOffset + 2], 2) << "deg ";
        oss << std::setw(4) << t_JointNames[2] << "=" << std::setw(6) << std::right << RoundFloat(p_ErgoData.data[t_DataOffset + 3], 2) << "deg";
        oss << "\n";
        
        t_JointNames = t_FingerJointNames;
        t_DataOffset += 4;
    }

    return oss.str();
}

float CalibrationApp::RoundFloat(float p_Value, int p_Decimals)
{
    float t_Power = std::pow(10.0f, static_cast<float>(p_Decimals));
    return std::round(p_Value * t_Power) / t_Power;
}

// SDK回调实现
void CalibrationApp::OnConnectedCallback(const ManusHost* const p_Host)
{
    if (s_Instance)
    {
        s_Instance->m_IsConnected = true;
        ClientLog::print("Connected to Core");
    }
}

void CalibrationApp::OnDisconnectedCallback(const ManusHost* const p_Host)
{
    if (s_Instance)
    {
        s_Instance->m_IsConnected = false;
        ClientLog::print("Disconnected from Core");
    }
}

void CalibrationApp::OnLogCallback(LogSeverity p_Severity, const char* const p_Log, uint32_t p_Length)
{
    // 可以在这里处理日志
}

void CalibrationApp::OnLandscapeCallback(const Landscape* const p_Landscape)
{
    if (s_Instance)
    {
        Landscape* t_NewLandscape = new Landscape();
        *t_NewLandscape = *p_Landscape;
        
        s_Instance->m_LandscapeMutex.lock();
        s_Instance->m_NewLandscape = t_NewLandscape;
        s_Instance->m_LandscapeMutex.unlock();
    }
}

void CalibrationApp::OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo)
{
    if (!s_Instance) return;

    s_Instance->m_ErgoMutex.lock();
    
    for (uint32_t i = 0; i < p_Ergo->dataCount; i++)
    {
        // Skip user ID data (as in original SDKClient)
        if (p_Ergo->data[i].isUserID) continue;

        ErgonomicsData* t_Ergo = nullptr;
        if (p_Ergo->data[i].id == s_Instance->m_FirstLeftGloveID && s_Instance->m_FirstLeftGloveID != 0)
        {
            t_Ergo = &s_Instance->m_LeftGloveErgoData;
        }
        if (p_Ergo->data[i].id == s_Instance->m_FirstRightGloveID && s_Instance->m_FirstRightGloveID != 0)
        {
            t_Ergo = &s_Instance->m_RightGloveErgoData;
        }
        
        if (t_Ergo != nullptr)
        {
            CoreSdk_GetTimestampInfo(p_Ergo->publishTime, &s_Instance->m_ErgoTimestampInfo);
            t_Ergo->id = p_Ergo->data[i].id;
            t_Ergo->isUserID = p_Ergo->data[i].isUserID;
            for (int j = 0; j < ErgonomicsDataType::ErgonomicsDataType_MAX_SIZE; j++)
            {
                t_Ergo->data[j] = p_Ergo->data[i].data[j];
            }
        }
    }
    
    s_Instance->m_ErgoMutex.unlock();
}

void CalibrationApp::OnSystemCallback(const SystemMessage* const p_SystemMessage)
{
    // 可以在这里处理系统消息
}

void CalibrationApp::HandleKeyboardInput()
{
    // 检测F5键
    static bool f5_was_pressed = false;
    bool f5_now_pressed = glfwGetKey(m_Window, GLFW_KEY_F5) == GLFW_PRESS;
    if (f5_now_pressed && !f5_was_pressed)
    {
        m_F5Pressed = true;
    }
    f5_was_pressed = f5_now_pressed;

    // 检测F9键
    static bool f9_was_pressed = false;
    bool f9_now_pressed = glfwGetKey(m_Window, GLFW_KEY_F9) == GLFW_PRESS;
    if (f9_now_pressed && !f9_was_pressed)
    {
        m_F9Pressed = true;
    }
    f9_was_pressed = f9_now_pressed;
}

bool CalibrationApp::SaveCalibrationToFile(uint32_t p_GloveId, const std::string& p_Hand)
{
    if (p_GloveId == 0)
    {
        ClientLog::print("[DEBUG] Invalid glove ID for saving calibration");
        return false;
    }
    
    // Get calibration data size
    uint32_t t_Size = 0;
    SDKReturnCode t_Res = CoreSdk_GetGloveCalibrationSize(p_GloveId, &t_Size);
    if (t_Res != SDKReturnCode_Success || t_Size == 0)
    {
        ClientLog::print("[DEBUG] Failed to get calibration size or size is 0");
        return false;
    }
    
    // Get calibration data
    unsigned char* t_CalibrationBytes = new unsigned char[t_Size];
    t_Res = CoreSdk_GetGloveCalibration(t_CalibrationBytes, t_Size);
    if (t_Res != SDKReturnCode_Success)
    {
        delete[] t_CalibrationBytes;
        ClientLog::print("[DEBUG] Failed to get calibration data");
        return false;
    }
    
    // Get current working directory
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == nullptr)
    {
        delete[] t_CalibrationBytes;
        ClientLog::print("[DEBUG] Failed to get current working directory");
        return false;
    }
    
    // Create file path
    std::string filename = std::string(cwd) + "/Calibration_" + p_Hand + ".mcal";
    
    // Write to file
    std::ofstream t_File(filename, std::ios::binary);
    if (!t_File.is_open())
    {
        delete[] t_CalibrationBytes;
        ClientLog::print("[DEBUG] Failed to open file for writing: {}", filename);
        return false;
    }
    
    t_File.write((char*)t_CalibrationBytes, t_Size);
    t_File.close();
    delete[] t_CalibrationBytes;
    
    ClientLog::print("[DEBUG] Calibration saved successfully to: {}", filename);
    return true;
}


