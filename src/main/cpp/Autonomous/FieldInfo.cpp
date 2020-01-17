#include <Autonomous/FieldInfo.h>
#include "frc/DriverStation.h"
#include <iostream>
#include <string>

FieldInfo::FieldInfo(std::string gameData) {
	ParseGameData(gameData);
}

FieldInfo::FieldInfo() {
	std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	ParseGameData(gameData);
}

FieldInfo::~FieldInfo() {
	// TODO Auto-generated destructor stub
}

void FieldInfo::ParseGameData(std::string gameData) {
	if(gameData.length() == 3)
	{
		switchLocation = gameData[0] == 'L' ? Location::Left : Location::Right;
		scaleLocation = gameData[1] == 'L' ? Location::Left : Location::Right;
		farSwitchLocation = gameData[2] == 'L' ? Location::Left : Location::Right;
	} else {
		std::cerr << "Did not receive correct game specific message to parse [" << gameData << "]";
		switchLocation = scaleLocation = farSwitchLocation = Location::Unknown;
	}
}

