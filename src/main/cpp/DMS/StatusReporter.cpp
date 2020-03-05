#include <iomanip>
#include <iostream>
#include <sstream>
#include "Robot.h"
#include "DMS/StatusReporter.h"
#include <frc/DriverStation.h>
#include <frc/RobotState.h>

using namespace std;

namespace StatusReporterUtil {
	static uint8_t map(double x, double in_min, double in_max, uint8_t out_min, uint8_t out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}

void StatusReporter::Launch() {
	std::cout << "StatusReporter::Launch...";
	reporterThread = std::thread(&StatusReporter::Run, this);
	reporterThread.detach();
	std::cout << "done\n";
}

StatusReporter::StatusReporter() {
	serial.reset(
		new frc::SerialPort(
			115200,
			frc::SerialPort::Port::kUSB1,
			8,
			frc::SerialPort::Parity::kParity_None,
			frc::SerialPort::StopBits::kStopBits_One));
	if (serial.get()) {
		running = true;
	} else {
		std::cout << "!!! Unable to start serial communications !!!\n";
	}

}


void StatusReporter::Run() {
	//frc::SetCurrentThreadPriority(true, 10); FIXME: Linker error 2019

	while (running) {
		try {
			SendData();
		} catch(...) {
			std::cout << "StatusReporter exited\n";
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::cout << "StatusReporter::Run exited\n";
}

void StatusReporter::SendData() {
	const double x = Robot::driveBase->GetLastSpeedX();
	const double y = Robot::driveBase->GetLastSpeedY();
	const double speed = sqrt(fabs(x*x) + fabs(y*y));

	const DriverStation::Alliance alliance =  DriverStation::GetInstance().GetAlliance();
	int allianceColor = 0;
	if (DriverStation::Alliance::kRed == alliance) {
		allianceColor = 1;
	} else if (DriverStation::Alliance::kBlue == alliance) {
		allianceColor = 2;
	}

	int robotState = 0;
	if (RobotState::IsDisabled()) {
		robotState = 1;
	} else if (RobotState::IsAutonomous()) {
		robotState = 2;
	} else if (RobotState::IsOperatorControl()) {
		robotState = 3;
	}

	const int DATA_SIZE = 14;
	char data[DATA_SIZE];
	data[0]  = (char) 254;
	data[1]  = (char) (dmsMode) ? driveStatus.FL : 0;
	data[2]  = (char) (dmsMode) ? steerStatus.FL : 0;
	data[3]  = (char) (dmsMode) ? driveStatus.FR : 0;
	data[4]  = (char) (dmsMode) ? steerStatus.FR : 0;
	data[5]  = (char) (dmsMode) ? driveStatus.RL : 0;
	data[6]  = (char) (dmsMode) ? steerStatus.RL : 0;
	data[7]  = (char) (dmsMode) ? driveStatus.RR : 0;
	data[8]  = (char) (dmsMode) ? steerStatus.RR : 0;
	// Custom Codes
	data[9] = Robot::turret->IsShooterEnabled();
	data[10] = (char) StatusReporterUtil::map(speed, 0.0, 1.0, 0, 250);
	data[11] = (char) allianceColor;	// 1 red, 2 blue, 0 unknown
	data[12] = (char) robotState;		// 0 - none, 1 - disabled, 2- auto, 3- tele
	// data[13] = (char) GetWheelColor   	// 0 - nothing, 1 - looking, 2 - 3 - 4 - 5 -		FIXME: Add Color Subsystem Hook, - when out of wheel mode
	int val = (((int)frc::Timer::GetFPGATimestamp()) % 5);
	// data[13] = (char) val;
	data[13] = 0;

	int written = serial->Write(data, DATA_SIZE);
	// std::cout << "StatusReporter - wrote " << written << " bytes\n";
	serial->Flush();
}
