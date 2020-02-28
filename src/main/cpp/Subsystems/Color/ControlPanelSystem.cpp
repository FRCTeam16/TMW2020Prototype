#include "Subsystems/Color/ControlPanelSystem.h"
#include <iostream>
#include <frc/Timer.h>
#include "Robot.h"

ControlPanelSystem::ControlPanelSystem()
{
    std::cout << "ControlPanelSystem initialized\n";
}

ControlPanelSystem::~ControlPanelSystem()
{
}

void ControlPanelSystem::Run() 
{
    /**
     * Enable color wheel mode from OI
     * Wait until we see a color for some period of time
     * if rotating wheel
     *  - run intake at rotate speed for some period of time
     *  - stop, disable and transition to next state
     * if rotating to color
     *  - calculate color counts
     *  - run intake until we meet color counts
     *  - stop, disable and transition to next state
     * 
     *  TODO: Think about how to recover/reset state
     */
    const double now = frc::Timer::GetFPGATimestamp();
    WheelColor gameDataColor = gameDataParser.Parse();
    const WheelColor color = colorSensor.ReadColor();
    
    if (runMode != Mode::kNone) {
        if (runningMode)
        {
            if (runMode == Mode::kRotateWheel)
            {
                RunRotateWheel(now, color);
            } else {
                // rotate to color
            }    
        } else {
            WaitForColorStart(now, color);
        }
    }
}

void ControlPanelSystem::RunRotateWheel(double now, WheelColor color)
{
    if (color == history.color) {
        rotateWheelData.currentRotateCount++;
    }

    // Check stop conditions
    if (rotateWheelData.currentRotateCount == 3) {
        runningMode = false;
    }
}

void ControlPanelSystem::RunRotateToColor(double now, WheelColor color)
{
    
}


void ControlPanelSystem::WaitForColorStart(double now, WheelColor color)
{
    // Waiting for color start
    if (WheelColor::Unknown != color) {
        if (history.firstSeen == -1) {
            // First time we've seen the color, record it
            history.color = color;
            history.firstSeen = now;
            std::cout << "ControlPanelSystem::Run() saw color " << color << " at now\n";
        } else if ((now - history.firstSeen) > kInitialColorTime) {
            runningMode = true;
            Robot::feederArm->StartIntakeForColorSpin();
            if (runMode == Mode::kRotateWheel) {
                rotateWheelData.lastColor = color;
            } else {
                // initialize initial rotate color state
            }


        } else {
            // waiting on initial color quieting time
        }
    } else {
        // We don't recognize a color sensor yet
        history.color = WheelColor::Unknown;
        history.firstSeen = -1;
    }
}

void ControlPanelSystem::SetMode(Mode _runMode)
{
    runMode = _runMode;

    if (runMode == Mode::kRotateWheel) {
        rotateWheelData.lastColor = WheelColor::Unknown;
        rotateWheelData.currentRotateCount = 0;
    }
}


void ControlPanelSystem::Instrument()
{
    // DEBUG ONLY
    colorSensor.ReadColor();
}