#ifndef _HAPTIC_FEEDBACK_HANDLER_HPP_
#define _HAPTIC_FEEDBACK_HANDLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>

#include "HapticConfig.hpp"
#include "ManusSDKTypes.h"

/// Per-finger force tier for WAVE -> Manus haptic mapping (with hysteresis).
enum class HapticForceTier : int
{
	Off = 0,
	Low = 1,   // small force: medium-frequency, low amplitude
	Mid = 2,   // medium force: medium-frequency, medium amplitude
	High = 3,  // large force: high-frequency, high amplitude
};

/// Maps WAVE force magnitudes (N) to tiered PWM vibration per finger.
/// Thread-safe: ZMQ thread writes forces; main/haptic thread calls Tick().
class HapticFeedbackHandler
{
public:
	static constexpr int NUM_FINGERS = NUM_FINGERS_ON_HAND;

	HapticFeedbackHandler();

	void ApplyConfig(const HapticConfig& p_Config);
	const HapticConfig& GetConfig() const { return m_Config; }

	void SetEnabled(bool p_Enabled);
	bool IsEnabled() const;

	void SetDebugPrint(bool p_Debug);
	bool IsDebugPrint() const;

	/// Latest per-finger |F| from retargeting (Newtons). Thumb..Pinky order.
	void UpdateForceMagnitudes(const float p_Left[NUM_FINGERS], const float p_Right[NUM_FINGERS]);

	/// Run PWM for one hand; returns powers sent (0 if skipped / no haptics glove).
	bool ComputeHandPowers(Side p_Side, float p_OutPowers[NUM_FINGERS]);

	/// Call after ComputeHandPowers to log tier summary (rate-limited).
	void MaybePrintDebugStatus();

	/// Synthetic test pattern (bypasses ZMQ) — index finger ramp for haptics bring-up.
	void RunSyntheticTierTest(float p_TimeSeconds);

	uint64_t GetReceivedMessageCount() const { return m_ReceivedMessageCount.load(); }
	uint64_t GetTickCount() const { return m_TickCount.load(); }

private:
	HapticForceTier ClassifyForce(float p_ForceN, HapticForceTier p_PreviousTier) const;
	HapticForceTier ClassifyFinger(float p_ForceN, Side p_Side, int p_FingerIndex);
	float SamplePwmPower(HapticForceTier p_Tier, Side p_Side, int p_FingerIndex, double p_NowSeconds);
	void ResetHandTiers(Side p_Side);
	bool IsHandBelowDeadzone(const std::array<float, NUM_FINGERS>& p_Forces) const;

	HapticConfig m_Config;
	bool m_Enabled{true};
	bool m_DebugPrint{false};

	std::mutex m_ForceMutex;
	std::array<float, NUM_FINGERS> m_LeftForceN{};
	std::array<float, NUM_FINGERS> m_RightForceN{};

	std::array<HapticForceTier, NUM_FINGERS> m_LeftTier{};
	std::array<HapticForceTier, NUM_FINGERS> m_RightTier{};
	std::array<double, NUM_FINGERS> m_LeftPhaseOrigin{};
	std::array<double, NUM_FINGERS> m_RightPhaseOrigin{};

	std::chrono::steady_clock::time_point m_LastDebugPrintTime{};
	std::atomic<uint64_t> m_ReceivedMessageCount{0};
	std::atomic<uint64_t> m_TickCount{0};
};

#endif
