#pragma once

#include "Subsystems/SubsystemManager.h"
#include "Subsystems/Color/ColorSensor.h"
#include "Subsystems/Color/GameDataParser.h"
#include <units/units.h>


struct ColorHistory {
	WheelColor color;
	units::second_t firstSeen;
};

struct RotateWheelData
{
	WheelColor lastColor = WheelColor::Unknown;
	int currentRotateCount = 0;
	bool passedColor = false;
};

struct RotateColorData
{
	WheelColor targetColor = WheelColor::Unknown;
	units::second_t foundTargetTime = -1_s;
};

class ControlPanelSystem : public SubsystemManager {
public:
    ControlPanelSystem();
	~ControlPanelSystem();

	enum class Mode { kNone, kRotateWheel, kRotateToColor };

	// virtual void Init() {}
	void Run() override;
	void Instrument() override;

	void SetMode(Mode _runMode);

private:
	GameDataParser gameDataParser;
    ColorSensor colorSensor;
	Mode runMode = Mode::kNone;
	bool foundColor = false;
	ColorHistory history {WheelColor::Unknown, -1_s};
	const units::second_t kInitialColorTime = 0.5_s;

	RotateWheelData rotateWheelData;
	const int kNumberRotations = 7;

	RotateColorData rotateColorData;
	const units::second_t postFindSpinTime = 0.1_s;

	void ResetState();
	void WaitForColorStart(units::second_t now, WheelColor color);
	void RunRotateWheel(units::second_t now, WheelColor color);
	void RunRotateToColor(units::second_t now, WheelColor color);
	WheelColor ComputeTargetColor();
	void HaltRotation();

};