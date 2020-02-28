#pragma once

#include "Subsystems/SubsystemManager.h"
#include "Subsystems/Color/ColorSensor.h"
#include "Subsystems/Color/GameDataParser.h"

struct ColorHistory {
	WheelColor color;
	double firstSeen;
};

struct RotateWheelData
{
	WheelColor lastColor = WheelColor::Unknown;
	int currentRotateCount = 0;
	bool pastInitialColor = false;
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
	bool runningMode = false;
	ColorHistory history {WheelColor::Unknown, -1};
	const double kInitialColorTime = 0.5;

	RotateWheelData rotateWheelData;

	void ResetState();
	void WaitForColorStart(double now, WheelColor color);
	void RunRotateWheel(double now, WheelColor color);
	void RunRotateToColor(double now, WheelColor color);
};