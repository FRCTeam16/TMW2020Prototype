#pragma once
#include <frc/DriverStation.h>
#include "Subsystems/Color/WheelColor.h"

class GameDataParser
{
public:
    WheelColor Parse()
    {
        std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
        WheelColor wheelColor = WheelColor::Unknown;
        if (gameData.length() > 0)
        {
            switch (gameData[0])
            {
            case 'B':
                wheelColor = WheelColor::Blue;
                break;
            case 'G':
                wheelColor = WheelColor::Green;
                break;
            case 'R':
                wheelColor = WheelColor::Red;
                break;
            case 'Y':
                wheelColor = WheelColor::Yellow;
                break;
            default:
                //This is corrupt data
                break;
            }
        }
        return wheelColor; 
    }
};
