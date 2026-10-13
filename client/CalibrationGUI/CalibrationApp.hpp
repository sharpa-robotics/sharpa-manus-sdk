#ifndef _CALIBRATION_APP_HPP_
#define _CALIBRATION_APP_HPP_

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <unordered_map>

#include "ManusSDK.h"

// Forward declarations
struct GLFWwindow;

/// @brief 标定应用主类
class CalibrationApp
{
public:
    CalibrationApp();
    ~CalibrationApp();

    /// @brief 初始化应用
    bool Initialize();

    /// @brief 运行主循环
    void Run();

    /// @brief 清理资源
    void Shutdown();

    // SDK回调函数
    static void OnConnectedCallback(const ManusHost* const p_Host);
    static void OnDisconnectedCallback(const ManusHost* const p_Host);
    static void OnLogCallback(LogSeverity p_Severity, const char* const p_Log, uint32_t p_Length);
    static void OnLandscapeCallback(const Landscape* const p_Landscape);
    static void OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo);
    static void OnSystemCallback(const SystemMessage* const p_SystemMessage);

private:
    /// @brief 初始化SDK
    bool InitializeSDK();

    /// @brief 连接Core
    bool ConnectToCore();

    /// @brief 更新数据
    void UpdateData();

    /// @brief 渲染GUI
    void RenderGUI();

    /// @brief 渲染顶部状态栏
    void RenderStatusBar();

    /// @brief 渲染右侧按钮面板
    void RenderButtonPanel();

    /// @brief 渲染标定信息
    void RenderCalibrationInfo();

    /// @brief 处理标定逻辑
    void HandleCalibration();

    /// @brief 开始标定
    void StartCalibration();

    /// @brief 执行标定步骤
    void ExecuteCalibrationStep();

    /// @brief 切换到下一个步骤
    void NextStep();

    /// @brief 重启标定
    void RestartCalibration();

    /// @brief 获取手套连接状态
    bool IsGloveConnected(uint32_t p_GloveId);

    /// @brief 获取手套关节角度字符串
    std::string GetJointAnglesString(const ErgonomicsData& p_ErgoData, const std::string& p_Hand = "left");

    /// @brief 四舍五入浮点数
    float RoundFloat(float p_Value, int p_Decimals);

    static CalibrationApp* s_Instance;

    // GLFW窗口
    GLFWwindow* m_Window = nullptr;

    // SDK相关
    bool m_IsConnected = false;

    // Landscape数据
    std::mutex m_LandscapeMutex;
    Landscape* m_Landscape = nullptr;
    Landscape* m_NewLandscape = nullptr;

    // 手套数据
    uint32_t m_FirstLeftGloveID = 0;
    uint32_t m_FirstRightGloveID = 0;
    
    std::mutex m_ErgoMutex;
    ErgonomicsData m_LeftGloveErgoData;
    ErgonomicsData m_RightGloveErgoData;
    ManusTimestampInfo m_ErgoTimestampInfo;

    // 标定相关
    bool m_CalibrateLeftHand = true;
    uint32_t m_CalibrationStep = 0;
    uint32_t m_CalibrationGloveId = 0;
    uint32_t m_NumberOfCalibrationSteps = 0;
    bool m_IsCalibrationInProgress = false;
    bool m_CalibrationStarted = false;
    std::string m_CalibrationMessage = "";
    GloveCalibrationStepData m_StepData;
    std::thread m_CalibrationThread;
    int m_CalibrationCountdown = -1; // 倒计时秒数，-1表示不在倒计时
    std::atomic<bool> m_ShouldStopCalibration{false}; // 标志：是否应该停止当前标定

    // UI状态
    bool m_ShowDemo = false;
    
    // 键盘状态
    bool m_F5Pressed = false;
    bool m_F9Pressed = false;
    
    /// @brief 处理键盘输入
    void HandleKeyboardInput();
    
    /// @brief 保存标定结果到文件
    bool SaveCalibrationToFile(uint32_t p_GloveId, const std::string& p_Hand);

    void UpdateResolvedGloveIds();
    void EnsureAutoUserAssignment();
    void ProcessGlovePairing();

    static bool IsKnownDongleId(const Landscape* p_Landscape, uint32_t p_DongleId);
    static bool IsGloveInDongleSlot(const Landscape* p_Landscape, uint32_t p_GloveId);
    static bool GloveNeedsDongleRepair(const GloveLandscapeData& p_Glove, const Landscape* p_Landscape);
    static uint32_t FindTargetDongleForGlove(const Landscape* p_Landscape, const GloveLandscapeData& p_Glove);

    std::chrono::steady_clock::time_point m_LastPairingTick{};
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> m_LastPairAttemptByGlove;
    bool m_AutoUserAssignmentEnabled = false;
};

#endif // _CALIBRATION_APP_HPP_

