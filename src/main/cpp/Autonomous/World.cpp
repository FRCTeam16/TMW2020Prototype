#include <frc/DriverStation.h>
#include "Autonomous/World.h"
#include <frc/Timer.h>


World::World() {
	frc::DriverStation::Alliance alliance = frc::DriverStation::GetInstance().GetAlliance();
	isRed = frc::DriverStation::Alliance::kRed == alliance;
	// fieldInfo = FieldInfo();
}


void World::Init() {
}

double World::GetClock() const {
	return frc::Timer::GetFPGATimestamp();
}

bool World::IsRed() {
	return isRed;
}


