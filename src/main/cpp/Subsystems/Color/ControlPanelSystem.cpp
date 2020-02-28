#include "Subsystems/Color/ControlPanelSystem.h"
#include <iostream>
#include <frc/Timer.h>
#include "Robot.h"

#define DEBUG 1

#ifdef DEBUG
#define D(x) x
#define DM(x) (std::cout << x << std::endl)
#else
#define D(x)
#define DM(x)
#endif

ControlPanelSystem::ControlPanelSystem()
{
    std::cout << "ControlPanelSystem initialized\n";
}

ControlPanelSystem::~ControlPanelSystem()
{
}

void ControlPanelSystem::Run()
{
    const units::second_t now = units::second_t(frc::Timer::GetFPGATimestamp());
    const WheelColor currentColor = colorSensor.ReadColor();

    if (runMode != Mode::kNone)
    {
        if (foundColor)
        {
            if (runMode == Mode::kRotateWheel)
            {
                RunRotateWheel(now, currentColor);
            }
            else if ((runMode == Mode::kRotateToColor) && (rotateColorData.targetColor != WheelColor::Unknown))
            {
                RunRotateToColor(now, currentColor);
            }
        }
        else
        {
            WaitForColorStart(now, currentColor);
        }
    }
}

void ControlPanelSystem::RunRotateWheel(units::second_t now, WheelColor color)
{
    if (color == history.color)
    {
        if (rotateWheelData.passedColor)
        {
            rotateWheelData.currentRotateCount++;
            rotateWheelData.passedColor = false;
            D(std::cout << "ControlPanelSystem::RunRotateWheels - passed color and counted revolution: " << rotateWheelData.currentRotateCount << std::endl);
        }
    }
    else
    {
        rotateWheelData.passedColor = true;
    }

    // Check stop conditions
    if (rotateWheelData.currentRotateCount >= kNumberRotations)
    {
        std::cout << "ControlPanelSystem::RunRotateWheel finished rotations\n";
        HaltRotation();
        
    }
}

void ControlPanelSystem::RunRotateToColor(units::second_t now, WheelColor color)
{
    std::cout << "ControlPanelSystem::RunRotateToColor\n";
    // Check Stop Condition
    if (rotateColorData.targetColor == color) {
        if ((now - rotateColorData.foundTargetTime) > postFindSpinTime) {
            std::cout << "ControlPanelSystem::RunRotateToColor found target color, halting\n";
            HaltRotation();
        } else {
            // found but waiting
        }
    }
}

void ControlPanelSystem::WaitForColorStart(units::second_t now, WheelColor color)
{
    // Waiting for color start
    if (WheelColor::Unknown != color)
    {
        if (history.firstSeen < 0_s)
        {
            // First time we've seen the color, record it
            history.color = color;
            history.firstSeen = now;
            std::cout << "ControlPanelSystem::Run() saw color " << color << " at " << now << "\n";
        }
        else if ((now - history.firstSeen) > kInitialColorTime)
        {
            std::cout << "ControlPanelSystem::WaitForColorStart - wait finished, setting found color true\n";
            foundColor = true;

            if (runMode == Mode::kRotateWheel)
            {
                double speed = BSPrefs::GetInstance()->GetDouble("FeederArm.IntakeSpeed.ColorWheel.Position", 1.0);
                Robot::feederArm->StartIntakeForColorSpin(speed);
                rotateWheelData.lastColor = color;
            }
            else
            {
                double speed = BSPrefs::GetInstance()->GetDouble("FeederArm.IntakeSpeed.ColorWheel.Color", 0.5);
                Robot::feederArm->StartIntakeForColorSpin(speed);
                rotateColorData.targetColor = ComputeTargetColor();
            }
        }
        else
        {
            // waiting on initial color quieting time
            DM("ControlPanelSystem::WaitForColorStart - Waiting on initial color quieting time");
        }
    }
    else
    {
        // We don't recognize a color sensor yet
        history = {};
    }
}

void ControlPanelSystem::HaltRotation()
{
    std::cout << "ControlPanelSystem::HaltRotation\n";
    Robot::feederArm->StopIntakeForColorSpin();
    runMode = Mode::kNone;
    foundColor = false;
    history = {};
    rotateWheelData = {};
    rotateColorData = {};
}

WheelColor ControlPanelSystem::ComputeTargetColor()
{
    WheelColor goal = gameDataParser.Parse();
    std::cout << "ControlPanelSystem::ComputeTargetColor - parsed " << goal << "\n";

    WheelColor target = WheelColor::Unknown;
    switch(goal)
    {
        case WheelColor::Blue:
            target = WheelColor::Red;
            break;
        case WheelColor::Yellow:
            target = WheelColor::Green;
            break;
        case WheelColor::Red:
            target = WheelColor::Blue;
            break;
        case WheelColor::Green:
            target = WheelColor::Yellow;
            break;
        case WheelColor::Unknown:
            std::cerr << "!!! ControlPanelSystem::ComputeTargetColor - Trying to parse Unknown target !!!\n";
            target = WheelColor::Unknown;
            break;
    }
    return target;
}

void ControlPanelSystem::SetMode(Mode _runMode)
{
    std::cout << "ControlPanelSystem::SetMode\n";
    runMode = _runMode;
    if (runMode == Mode::kNone) {
        HaltRotation();
    }
}

void ControlPanelSystem::Instrument()
{
    // DEBUG ONLY
    colorSensor.ReadColor();
}