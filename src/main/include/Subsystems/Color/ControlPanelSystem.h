#pragma once

#include "Subsystems/SubsystemManager.h"
#include "Subsystems/Color/ColorSensor.h"

struct ColorHistory {
	WheelColor color;
	double firstSeen;
};

class ControlPanelSystem : public SubsystemManager {
public:
    ControlPanelSystem();
	~ControlPanelSystem();
	// virtual void Init() {}
	void Run() override;
	void Instrument() override;

private:
	enum Mode { RotateWheel, RotateToColor, Finished };
    ColorSensor colorSensor;
	bool enabled = false;
	Mode mode = Mode::RotateWheel;
	ColorHistory history {WheelColor::Unknown, -1};
	const double kInitialColorTime = 0.5;
};