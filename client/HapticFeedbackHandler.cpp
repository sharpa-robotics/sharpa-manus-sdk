#include "HapticFeedbackHandler.hpp"

#include "ClientLogging.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <thread>

using ManusSDK::ClientLog;

HapticFeedbackHandler::HapticFeedbackHandler()
{
	ApplyConfig(HapticConfig::LoadDefaults());
}

void HapticFeedbackHandler::ApplyConfig(const HapticConfig& p_Config)
{
	m_Config = p_Config;
	const auto t_Now = std::chrono::steady_clock::now();
	const double t_Sec = std::chrono::duration<double>(t_Now.time_since_epoch()).count();
	for (int i = 0; i < NUM_FINGERS; ++i)
	{
		m_LeftPhaseOrigin[i] = t_Sec + m_Config.fingerPhaseOffsetSec * i;
		m_RightPhaseOrigin[i] = t_Sec + m_Config.fingerPhaseOffsetSec * i + 0.05;
	}
	m_LastDebugPrintTime = t_Now;
}

void HapticFeedbackHandler::SetEnabled(bool p_Enabled)
{
	m_Enabled = p_Enabled;
	ClientLog::print("[Haptics] WAVE feedback path {}", m_Enabled ? "ENABLED" : "DISABLED");
}

bool HapticFeedbackHandler::IsEnabled() const
{
	return m_Enabled;
}

void HapticFeedbackHandler::SetDebugPrint(bool p_Debug)
{
	m_DebugPrint = p_Debug;
	ClientLog::print("[Haptics] Debug print {}", m_DebugPrint ? "ON" : "OFF");
}

bool HapticFeedbackHandler::IsDebugPrint() const
{
	return m_DebugPrint;
}

void HapticFeedbackHandler::UpdateForceMagnitudes(const float p_Left[NUM_FINGERS], const float p_Right[NUM_FINGERS])
{
	std::lock_guard<std::mutex> t_Lock(m_ForceMutex);
	std::memcpy(m_LeftForceN.data(), p_Left, sizeof(float) * NUM_FINGERS);
	std::memcpy(m_RightForceN.data(), p_Right, sizeof(float) * NUM_FINGERS);
	m_ReceivedMessageCount++;
}

HapticForceTier HapticFeedbackHandler::ClassifyForce(float p_ForceN, HapticForceTier p_PreviousTier) const
{
	if (p_ForceN < m_Config.deadzoneN)
	{
		return HapticForceTier::Off;
	}

	switch (p_PreviousTier)
	{
	case HapticForceTier::High:
		if (p_ForceN < m_Config.highExitN)
		{
			if (p_ForceN >= m_Config.midEnterN)
			{
				return HapticForceTier::Mid;
			}
			if (p_ForceN >= m_Config.lowEnterN)
			{
				return HapticForceTier::Low;
			}
			return HapticForceTier::Off;
		}
		return HapticForceTier::High;

	case HapticForceTier::Mid:
		if (p_ForceN >= m_Config.highEnterN)
		{
			return HapticForceTier::High;
		}
		if (p_ForceN < m_Config.midExitN)
		{
			if (p_ForceN >= m_Config.lowEnterN)
			{
				return HapticForceTier::Low;
			}
			return HapticForceTier::Off;
		}
		return HapticForceTier::Mid;

	case HapticForceTier::Low:
		if (p_ForceN >= m_Config.highEnterN)
		{
			return HapticForceTier::High;
		}
		if (p_ForceN >= m_Config.midEnterN)
		{
			return HapticForceTier::Mid;
		}
		if (p_ForceN < m_Config.lowExitN)
		{
			return HapticForceTier::Off;
		}
		return HapticForceTier::Low;

	default:
		if (p_ForceN >= m_Config.highEnterN)
		{
			return HapticForceTier::High;
		}
		if (p_ForceN >= m_Config.midEnterN)
		{
			return HapticForceTier::Mid;
		}
		if (p_ForceN >= m_Config.lowEnterN)
		{
			return HapticForceTier::Low;
		}
		return HapticForceTier::Off;
	}
}

HapticForceTier HapticFeedbackHandler::ClassifyFinger(float p_ForceN, Side p_Side, int p_FingerIndex)
{
	const HapticForceTier t_Prev = (p_Side == Side::Side_Left)
		? m_LeftTier[p_FingerIndex]
		: m_RightTier[p_FingerIndex];
	const HapticForceTier t_New = ClassifyForce(p_ForceN, t_Prev);
	if (p_Side == Side::Side_Left)
	{
		m_LeftTier[p_FingerIndex] = t_New;
	}
	else
	{
		m_RightTier[p_FingerIndex] = t_New;
	}
	return t_New;
}

float HapticFeedbackHandler::SamplePwmPower(HapticForceTier p_Tier, Side p_Side, int p_FingerIndex, double p_NowSeconds)
{
	if (p_Tier == HapticForceTier::Off)
	{
		return 0.0f;
	}

	const int t_TierIdx = static_cast<int>(p_Tier);
	const HapticTierParams& t_Params = m_Config.tierParams[t_TierIdx];
	const double t_Origin = (p_Side == Side::Side_Left)
		? m_LeftPhaseOrigin[p_FingerIndex]
		: m_RightPhaseOrigin[p_FingerIndex];

	const double t_Phase = std::fmod((p_NowSeconds - t_Origin) * t_Params.pulseHz, 1.0);
	if (t_Phase < 0.0)
	{
		return t_Params.amplitude;
	}
	return (t_Phase < static_cast<double>(t_Params.duty)) ? t_Params.amplitude : 0.0f;
}

void HapticFeedbackHandler::ResetHandTiers(Side p_Side)
{
	for (int i = 0; i < NUM_FINGERS; ++i)
	{
		if (p_Side == Side::Side_Left)
		{
			m_LeftTier[i] = HapticForceTier::Off;
		}
		else
		{
			m_RightTier[i] = HapticForceTier::Off;
		}
	}
}

bool HapticFeedbackHandler::IsHandBelowDeadzone(const std::array<float, NUM_FINGERS>& p_Forces) const
{
	for (int i = 0; i < NUM_FINGERS; ++i)
	{
		if (p_Forces[i] >= m_Config.deadzoneN)
		{
			return false;
		}
	}
	return true;
}

bool HapticFeedbackHandler::ComputeHandPowers(Side p_Side, float p_OutPowers[NUM_FINGERS])
{
	if (!m_Enabled)
	{
		std::memset(p_OutPowers, 0, sizeof(float) * NUM_FINGERS);
		return false;
	}

	std::array<float, NUM_FINGERS> t_Forces{};
	{
		std::lock_guard<std::mutex> t_Lock(m_ForceMutex);
		if (p_Side == Side::Side_Left)
		{
			t_Forces = m_LeftForceN;
		}
		else
		{
			t_Forces = m_RightForceN;
		}
	}

	if (IsHandBelowDeadzone(t_Forces))
	{
		ResetHandTiers(p_Side);
		std::memset(p_OutPowers, 0, sizeof(float) * NUM_FINGERS);
		m_TickCount++;
		return false;
	}

	const auto t_Now = std::chrono::steady_clock::now();
	const double t_Sec = std::chrono::duration<double>(t_Now.time_since_epoch()).count();

	bool t_AnyNonZero = false;
	for (int i = 0; i < NUM_FINGERS; ++i)
	{
		const HapticForceTier t_Tier = ClassifyFinger(t_Forces[i], p_Side, i);
		p_OutPowers[i] = SamplePwmPower(t_Tier, p_Side, i, t_Sec);
		if (p_OutPowers[i] > 0.0f)
		{
			t_AnyNonZero = true;
		}
	}

	m_TickCount++;
	return t_AnyNonZero;
}

void HapticFeedbackHandler::MaybePrintDebugStatus()
{
	if (!m_DebugPrint)
	{
		return;
	}

	const auto t_Now = std::chrono::steady_clock::now();
	const double t_Elapsed = std::chrono::duration<double>(t_Now - m_LastDebugPrintTime).count();
	if (t_Elapsed < m_Config.debugPrintIntervalSec)
	{
		return;
	}
	m_LastDebugPrintTime = t_Now;

	static const char* s_FingerNames[] = {"Th", "Ix", "Md", "Rg", "Pk"};
	static const char* s_TierNames[] = {"Off", "Low", "Mid", "High"};

	std::array<float, NUM_FINGERS> t_LeftF{};
	std::array<float, NUM_FINGERS> t_RightF{};
	{
		std::lock_guard<std::mutex> t_Lock(m_ForceMutex);
		t_LeftF = m_LeftForceN;
		t_RightF = m_RightForceN;
	}

	auto t_FormatHand = [&](Side p_Side, const std::array<float, NUM_FINGERS>& p_F) -> std::string
	{
		std::string t_Out;
		for (int i = 0; i < NUM_FINGERS; ++i)
		{
			const HapticForceTier t_Tier = (p_Side == Side::Side_Left) ? m_LeftTier[i] : m_RightTier[i];
			t_Out += std::string(s_FingerNames[i]) + "=" + std::to_string(p_F[i]).substr(0, 4) + "N(" +
				s_TierNames[static_cast<int>(t_Tier)] + ") ";
		}
		return t_Out;
	};

	ClientLog::info(
		"[Haptics] ZMQ_rx={} ticks={} | L: {}| R: {}",
		m_ReceivedMessageCount.load(),
		m_TickCount.load(),
		t_FormatHand(Side::Side_Left, t_LeftF),
		t_FormatHand(Side::Side_Right, t_RightF));
}

void HapticFeedbackHandler::RunSyntheticTierTest(float p_TimeSeconds)
{
	const float t_Step = p_TimeSeconds / 3.0f;
	float t_Left[NUM_FINGERS] = {0};
	float t_Right[NUM_FINGERS] = {0};

	ClientLog::print("[Haptics] Synthetic tier test: index finger Low/Mid/High (~{:.1f}s each)", t_Step);

	t_Left[1] = m_Config.syntheticLowForceN;
	UpdateForceMagnitudes(t_Left, t_Right);
	std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_Step * 1000)));

	t_Left[1] = m_Config.syntheticMidForceN;
	UpdateForceMagnitudes(t_Left, t_Right);
	std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_Step * 1000)));

	t_Left[1] = m_Config.syntheticHighForceN;
	UpdateForceMagnitudes(t_Left, t_Right);
	std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_Step * 1000)));

	std::memset(t_Left, 0, sizeof(t_Left));
	UpdateForceMagnitudes(t_Left, t_Right);
	ClientLog::print("[Haptics] Synthetic tier test finished.");
}
