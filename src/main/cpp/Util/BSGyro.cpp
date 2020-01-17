/*
 * BSGyro.cpp
 *
 *  Created on: Feb 27, 2017
 *      Author: User
 */
#include <iostream>
#include "Util/BSGyro.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

BSGyro::BSGyro(WPI_TalonSRX *talon) : pigeon(new PigeonIMU(talon)) {
	std::cout << "Constructed BSGyro attached to TalonSRX\n";
}

BSGyro::BSGyro(int canID) : pigeon(new PigeonIMU(canID)) {
	std::cout << "Constructed BSGyro with device ID " << canID << "\n";
}

BSGyro::~BSGyro() = default;

float BSGyro::GetOffset() {
	return offset;
}

void BSGyro::SetOffset(float _offset) {
	frc::SmartDashboard::PutNumber("BSGyro Set Offset", _offset);
	offset = _offset;
}

float BSGyro::GetYaw() {
	const double rawYaw = ReadYaw();
	if (rawYaw > 180.0) {
		return rawYaw - 360.0;
	} else if (rawYaw < -180.0) {
		return rawYaw + 360.0;
	} else {
		return rawYaw;
	}
}

double BSGyro::ReadYaw() {
	double ypr[3];
	int errorCode = pigeon->GetYawPitchRoll(ypr);
	if (ErrorCode::OK != static_cast<ErrorCode>(errorCode)) {
		//std::cerr << "Error from gyro: " << errorCode << "\n";
	}

	double trimmedYaw = fmod(ypr[0],360.0);

	SmartDashboard::PutNumber("YawOffset", GetOffset());
	// SmartDashboard::PutNumber("PigeonYaw", ypr[0]);
	SmartDashboard::PutNumber("TrimmedYaw", trimmedYaw);

	return GetOffset() + trimmedYaw;
}

double BSGyro::PIDGet() {
	return GetYaw();
}

PigeonIMU* BSGyro::GetPigeon() {
	return pigeon.get();
}

void BSGyro::ZeroYaw() {
	pigeon->SetYaw(0, 0);
}

