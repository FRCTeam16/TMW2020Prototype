#include "Subsystems/Color/ControlPanelSystem.h"
#include <iostream>
#include <frc/Timer.h>

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
    const WheelColor color = colorSensor.ReadColor();
    
    if (enabled) {
        // const WheelColor color = colorSensor.ReadColor();
        if (WheelColor::Unknown != color) {
            if (history.firstSeen = -1) {
                // First time we've seen the color, record it
                history.color = color;
                history.firstSeen = now;
            } else if ((now - history.firstSeen) > kInitialColorTime){
                // Begin intake operation
                // Robot::feederArm->StartIntakeForColorSpin
            } else {
                // waiting on initial color quieting time
            }
        } else {
            // We don't recognize a color sensor yet
            history.color = WheelColor::Unknown;
            history.firstSeen = -1;
        }
    }
}

void ControlPanelSystem::Instrument()
{
    // DEBUG ONLY
    colorSensor.ReadColor();
}