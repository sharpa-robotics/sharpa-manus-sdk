#ifndef _HAPTIC_CONFIG_HPP_
#define _HAPTIC_CONFIG_HPP_

#include <array>
#include <string>

struct HapticTierParams
{
	float amplitude = 0.0f;
	float pulseHz = 0.0f;
	float duty = 0.0f;
};

struct HapticConfig
{
	float deadzoneN = 0.25f;
	float lowEnterN = 0.6f;
	float lowExitN = 0.4f;
	float midEnterN = 2.5f;
	float midExitN = 2.0f;
	float highEnterN = 5.0f;
	float highExitN = 4.0f;
	std::array<HapticTierParams, 4> tierParams{};
	float fingerPhaseOffsetSec = 0.02f;
	float debugPrintIntervalSec = 1.0f;
	float syntheticLowForceN = 1.0f;
	float syntheticMidForceN = 3.0f;
	float syntheticHighForceN = 6.0f;
	float syntheticStepDurationSec = 2.0f;
	int pwmTickHz = 50;
	unsigned long long minMsBetweenHapticCommands = 20;
	int zeroBurstOnAllOff = 3;

	static HapticConfig LoadDefaults();
	static HapticConfig LoadFromFile(const std::string& p_Path);
	static HapticConfig LoadFromEnvOrDefault();
};

#endif
