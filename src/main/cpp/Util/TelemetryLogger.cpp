#include "Util/TelemetryLogger.h"
#include "Robot.h"
#include "RobotMap.h"
#include <iostream>
#include <ctime>
#include <frc/Threads.h>
#include "Subsystems/Drive/DriveInfo.h"

#define LOGNAME_FORMAT "log/%Y%m%d_%H%M%S"
#define LOGNAME_SIZE 20

TelemetryLogger::TelemetryLogger() = default;

TelemetryLogger::~TelemetryLogger() = default;

void TelemetryLogger::Launch() {
	std::cout << "TelemetryLogger::Launch...";
	this->telemetryThread = std::thread(&TelemetryLogger::Run, this);
	this->telemetryThread.detach();
	std::cout << "done\n";
}

void TelemetryLogger::Run() {
	// frc::SetCurrentThreadPriority(false, 10); FIXME: Linker error 2019

	while (true) {
		try {
			if (running) {
				Log();
			}
		} catch(...) {
			std::cout << "StatusReporter exited\n";
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}


void TelemetryLogger::Begin() {
	std::cout << "TelemetryLogger::Begin...";
    static char name[LOGNAME_SIZE];
    time_t now = time(0);
    strftime(name, sizeof(name), LOGNAME_FORMAT, localtime(&now));
	logFile.open(name, std::ios::out);

	logFile << "time, yaw, heading, compass, pitch, roll, ax, ay, az, "
			<< "fl.pos, fr.pos, rl.pos, rr.pos"
			<< "encoder.pos, encoder.error, setpoint"
			<< '\n';
	running = true;
	std::cout << "Writing to " << name << "\n";
}

void TelemetryLogger::End() {
	running = false;
	logFile.flush();
	logFile.close();
}

void TelemetryLogger::Log() {
	double ypr[3] = {0,0,0};
	short int xyz[3] = {0,0,0};
	double fusedHeading = 0; // = imu->GetFusedHeading();
	double compassHeading = 0; // = imu->GetCompassHeading()

	// FIXME: Update with alternate IMU if desired
	// PigeonIMU *imu = RobotMap::gyro->GetPigeon();
	// imu->GetBiasedAccelerometer(xyz);
	// imu->GetYawPitchRoll(ypr);

	DriveInfo<double> encoders = Robot::driveBase->GetDriveEncoderPositions();

	const char delimiter = ',';
	logFile << Timer::GetFPGATimestamp() << delimiter
			<< ypr[0] << delimiter
			<< fusedHeading << delimiter
			<< compassHeading << delimiter
			<< ypr[1] << delimiter
			<< ypr[2] << delimiter
			<< xyz[0] << delimiter
			<< xyz[1] << delimiter
			<< xyz[2] << delimiter
			<< encoders.FL << delimiter
			<< encoders.FR << delimiter
			<< encoders.RL << delimiter
			<< encoders.RR << delimiter
			<< Robot::driveBase->GetDriveControlEncoderPosition() << delimiter
			<< Robot::driveBase->GetDriveControlError() << delimiter
			<< Robot::driveBase->GetDriveControlSetpoint()
			<< '\n';
}
