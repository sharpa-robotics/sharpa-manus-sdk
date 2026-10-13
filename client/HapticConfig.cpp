#include "HapticConfig.hpp"

#include "ClientLogging.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits.h>
#include <sstream>
#include <unistd.h>
#include <vector>

using ManusSDK::ClientLog;

namespace
{
std::string Trim(const std::string& p_Text)
{
	const auto t_Start = p_Text.find_first_not_of(" \t\r\n");
	if (t_Start == std::string::npos)
	{
		return "";
	}
	const auto t_End = p_Text.find_last_not_of(" \t\r\n");
	return p_Text.substr(t_Start, t_End - t_Start + 1);
}

std::string StripComment(const std::string& p_Line)
{
	std::string t_Out;
	bool t_InSingle = false;
	bool t_InDouble = false;
	for (char t_Ch : p_Line)
	{
		if (t_Ch == '"' && !t_InSingle)
		{
			t_InDouble = !t_InDouble;
		}
		else if (t_Ch == '\'' && !t_InDouble)
		{
			t_InSingle = !t_InSingle;
		}
		if (t_Ch == '#' && !t_InSingle && !t_InDouble)
		{
			break;
		}
		t_Out.push_back(t_Ch);
	}
	return t_Out;
}

bool ParseFloat(const std::string& p_Text, float& p_Out)
{
	try
	{
		p_Out = std::stof(p_Text);
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool ParseInt(const std::string& p_Text, int& p_Out)
{
	try
	{
		p_Out = std::stoi(p_Text);
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool ParseUll(const std::string& p_Text, unsigned long long& p_Out)
{
	try
	{
		p_Out = std::stoull(p_Text);
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool FileExists(const std::string& p_Path)
{
	std::ifstream t_File(p_Path);
	return t_File.good();
}

/// Directory containing SharpaManusClient.out (Linux: /proc/self/exe).
std::string GetExecutableDirectory()
{
	char t_Path[PATH_MAX] = {};
	const ssize_t t_Len = readlink("/proc/self/exe", t_Path, sizeof(t_Path) - 1);
	if (t_Len <= 0)
	{
		return "";
	}
	t_Path[t_Len] = '\0';
	const std::filesystem::path t_Exe(t_Path);
	if (!t_Exe.has_parent_path())
	{
		return "";
	}
	return t_Exe.parent_path().string();
}

void AppendUniquePath(std::vector<std::string>& p_Candidates, const std::string& p_Path)
{
	if (p_Path.empty())
	{
		return;
	}
	for (const std::string& t_Existing : p_Candidates)
	{
		if (t_Existing == p_Path)
		{
			return;
		}
	}
	p_Candidates.push_back(p_Path);
}

void AppendRetargetingTreeCandidates(std::vector<std::string>& p_Candidates, const std::filesystem::path& p_ParentDir)
{
	if (p_ParentDir.empty() || !std::filesystem::exists(p_ParentDir))
	{
		return;
	}
	try
	{
		for (const auto& t_Entry : std::filesystem::directory_iterator(p_ParentDir))
		{
			if (!t_Entry.is_directory())
			{
				continue;
			}
			const std::string t_Name = t_Entry.path().filename().string();
			if (t_Name.rfind("retargeting_alg_release_V", 0) != 0)
			{
				continue;
			}
			AppendUniquePath(
				p_Candidates,
				(t_Entry.path() / "include" / "haptic_feedback.yaml").string());
		}
	}
	catch (...)
	{
		// Ignore unreadable parent directories.
	}
}

void AppendHapticConfigCandidates(std::vector<std::string>& p_Candidates, const std::filesystem::path& p_BaseDir)
{
	if (p_BaseDir.empty())
	{
		return;
	}
	AppendUniquePath(p_Candidates, (p_BaseDir / "include" / "haptic_feedback.yaml").string());
	AppendUniquePath(p_Candidates, (p_BaseDir / "haptic_feedback.yaml").string());

	// Sibling retargeting trees (typical repo layout next to Client).
	const std::filesystem::path t_Parent = p_BaseDir.parent_path();
	if (!t_Parent.empty())
	{
		AppendRetargetingTreeCandidates(p_Candidates, t_Parent);
		// Legacy explicit paths (kept for older checkouts).
		AppendUniquePath(
			p_Candidates,
			(t_Parent / "retargeting_alg_release_V3.2" / "retargeting_alg_release" / "include" /
			 "haptic_feedback.yaml")
				.string());
		AppendUniquePath(
			p_Candidates,
			(t_Parent / "retargeting_alg_release_V4.0" / "include" / "haptic_feedback.yaml").string());
		AppendUniquePath(
			p_Candidates,
			(t_Parent / "retargeting_alg_release_V5.0" / "include" / "haptic_feedback.yaml").string());
	}
}

void SetTierParams(HapticConfig& p_Config, int p_TierIdx, float p_Amp, float p_Hz, float p_Duty)
{
	if (p_TierIdx < 0 || p_TierIdx >= static_cast<int>(p_Config.tierParams.size()))
	{
		return;
	}
	p_Config.tierParams[p_TierIdx].amplitude = p_Amp;
	p_Config.tierParams[p_TierIdx].pulseHz = p_Hz;
	p_Config.tierParams[p_TierIdx].duty = p_Duty;
}

void ApplyKeyValue(HapticConfig& p_Config, const std::vector<std::string>& p_Sections, const std::string& p_Key, const std::string& p_Value)
{
	float t_Float = 0.0f;
	int t_Int = 0;
	unsigned long long t_Ull = 0;

	if (p_Sections.size() >= 1 && p_Sections[0] == "thresholds")
	{
		if (p_Sections.size() == 1)
		{
			if (p_Key == "deadzone_n" && ParseFloat(p_Value, t_Float))
			{
				p_Config.deadzoneN = t_Float;
			}
			return;
		}
		if (p_Sections.size() == 2)
		{
			const std::string& t_Tier = p_Sections[1];
			if (t_Tier == "low")
			{
				if (p_Key == "enter_n" && ParseFloat(p_Value, t_Float))
				{
					p_Config.lowEnterN = t_Float;
				}
				else if (p_Key == "exit_n" && ParseFloat(p_Value, t_Float))
				{
					p_Config.lowExitN = t_Float;
				}
			}
			else if (t_Tier == "mid")
			{
				if (p_Key == "enter_n" && ParseFloat(p_Value, t_Float))
				{
					p_Config.midEnterN = t_Float;
				}
				else if (p_Key == "exit_n" && ParseFloat(p_Value, t_Float))
				{
					p_Config.midExitN = t_Float;
				}
			}
			else if (t_Tier == "high")
			{
				if (p_Key == "enter_n" && ParseFloat(p_Value, t_Float))
				{
					p_Config.highEnterN = t_Float;
				}
				else if (p_Key == "exit_n" && ParseFloat(p_Value, t_Float))
				{
					p_Config.highExitN = t_Float;
				}
			}
		}
		return;
	}

	if (p_Sections.size() >= 2 && p_Sections[0] == "pwm")
	{
		int t_TierIdx = -1;
		if (p_Sections[1] == "low")
		{
			t_TierIdx = 1;
		}
		else if (p_Sections[1] == "mid")
		{
			t_TierIdx = 2;
		}
		else if (p_Sections[1] == "high")
		{
			t_TierIdx = 3;
		}
		if (t_TierIdx < 0)
		{
			return;
		}
		float t_Amp = p_Config.tierParams[t_TierIdx].amplitude;
		float t_Hz = p_Config.tierParams[t_TierIdx].pulseHz;
		float t_Duty = p_Config.tierParams[t_TierIdx].duty;
		if (p_Key == "amplitude" && ParseFloat(p_Value, t_Float))
		{
			t_Amp = t_Float;
		}
		else if (p_Key == "pulse_hz" && ParseFloat(p_Value, t_Float))
		{
			t_Hz = t_Float;
		}
		else if (p_Key == "duty" && ParseFloat(p_Value, t_Float))
		{
			t_Duty = t_Float;
		}
		else
		{
			return;
		}
		SetTierParams(p_Config, t_TierIdx, t_Amp, t_Hz, t_Duty);
		return;
	}

	if (p_Sections.size() == 1 && p_Sections[0] == "timing")
	{
		if (p_Key == "pwm_tick_hz" && ParseInt(p_Value, t_Int))
		{
			p_Config.pwmTickHz = t_Int;
		}
		else if (p_Key == "min_ms_between_haptic_commands" && ParseUll(p_Value, t_Ull))
		{
			p_Config.minMsBetweenHapticCommands = t_Ull;
		}
		else if (p_Key == "finger_phase_offset_sec" && ParseFloat(p_Value, t_Float))
		{
			p_Config.fingerPhaseOffsetSec = t_Float;
		}
		else if (p_Key == "debug_print_interval_sec" && ParseFloat(p_Value, t_Float))
		{
			p_Config.debugPrintIntervalSec = t_Float;
		}
		else if (p_Key == "zero_burst_on_all_off" && ParseInt(p_Value, t_Int))
		{
			p_Config.zeroBurstOnAllOff = t_Int;
		}
		return;
	}

	if (p_Sections.size() == 1 && p_Sections[0] == "synthetic_test")
	{
		if (p_Key == "step_duration_sec" && ParseFloat(p_Value, t_Float))
		{
			p_Config.syntheticStepDurationSec = t_Float;
		}
		else if (p_Key == "low_force_n" && ParseFloat(p_Value, t_Float))
		{
			p_Config.syntheticLowForceN = t_Float;
		}
		else if (p_Key == "mid_force_n" && ParseFloat(p_Value, t_Float))
		{
			p_Config.syntheticMidForceN = t_Float;
		}
		else if (p_Key == "high_force_n" && ParseFloat(p_Value, t_Float))
		{
			p_Config.syntheticHighForceN = t_Float;
		}
	}
}
} // namespace

HapticConfig HapticConfig::LoadDefaults()
{
	HapticConfig t_Config;
	SetTierParams(t_Config, 0, 0.0f, 0.0f, 0.0f);
	SetTierParams(t_Config, 1, 0.28f, 17.0f, 0.5f);
	SetTierParams(t_Config, 2, 0.52f, 17.0f, 0.5f);
	SetTierParams(t_Config, 3, 0.88f, 24.0f, 0.65f);
	return t_Config;
}

HapticConfig HapticConfig::LoadFromFile(const std::string& p_Path)
{
	HapticConfig t_Config = LoadDefaults();
	std::ifstream t_File(p_Path);
	if (!t_File.is_open())
	{
		ClientLog::warn("[Haptics] Config file not found: {} (using defaults)", p_Path);
		return t_Config;
	}

	std::vector<std::string> t_Sections;
	std::string t_Line;
	while (std::getline(t_File, t_Line))
	{
		t_Line = Trim(StripComment(t_Line));
		if (t_Line.empty())
		{
			continue;
		}

		if (t_Line.back() == ':')
		{
			const std::string t_Section = Trim(t_Line.substr(0, t_Line.size() - 1));
			if (!t_Section.empty())
			{
				if (t_Section == "thresholds" || t_Section == "pwm" || t_Section == "timing" ||
					t_Section == "synthetic_test" || t_Section == "bridge")
				{
					t_Sections = {t_Section};
				}
				else if (!t_Sections.empty() && (t_Sections[0] == "thresholds" || t_Sections[0] == "pwm"))
				{
					t_Sections.resize(1);
					t_Sections.push_back(t_Section);
				}
			}
			continue;
		}

		const auto t_Colon = t_Line.find(':');
		if (t_Colon == std::string::npos)
		{
			continue;
		}
		const std::string t_Key = Trim(t_Line.substr(0, t_Colon));
		const std::string t_Value = Trim(t_Line.substr(t_Colon + 1));
		if (t_Key.empty() || t_Value.empty())
		{
			continue;
		}
		ApplyKeyValue(t_Config, t_Sections, t_Key, t_Value);
	}

	ClientLog::print("[Haptics] Loaded config: {}", p_Path);
	return t_Config;
}

HapticConfig HapticConfig::LoadFromEnvOrDefault()
{
	std::vector<std::string> t_Candidates;

	// Optional override (custom install path).
	const char* t_Env = std::getenv("HAPTIC_FEEDBACK_CONFIG");
	if (t_Env != nullptr && t_Env[0] != '\0')
	{
		AppendUniquePath(t_Candidates, t_Env);
	}

	// Prefer config next to the binary — works regardless of shell cwd.
	AppendHapticConfigCandidates(t_Candidates, GetExecutableDirectory());

	// Fallback: paths relative to current working directory.
	AppendHapticConfigCandidates(t_Candidates, std::filesystem::current_path());
	AppendUniquePath(
		t_Candidates,
		"../retargeting_alg_release_V3.2/retargeting_alg_release/include/haptic_feedback.yaml");
	AppendUniquePath(
		t_Candidates,
		"../../retargeting_alg_release_V3.2/retargeting_alg_release/include/haptic_feedback.yaml");

	for (const std::string& t_Path : t_Candidates)
	{
		if (FileExists(t_Path))
		{
			return LoadFromFile(t_Path);
		}
	}

	ClientLog::warn(
		"[Haptics] No haptic_feedback.yaml found (expected <client>/include/haptic_feedback.yaml "
		"next to SharpaManusClient.out). Using built-in defaults. "
		"Optional override: export HAPTIC_FEEDBACK_CONFIG=/path/to/file");
	return LoadDefaults();
}
