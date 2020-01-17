#include <frc/smartdashboard/SmartDashboard.h>
#include "DMS/DmsProcessManager.h"
#include "Robot.h"

/**
0 - no updated data (purple)
1 - both amperage draw and encoder values were as expected (green light)
2 - amperage was outside (high or lower, but not 0) than expected, encoder value was expected (yellow/flashing)
3 - amperage as expected, encoder value was outside of expected range (but not 0) (yellow/solid)
4 - amperage draw was 0 (red/flashing)
5 - encoder value was 0 (red
 */


inline void add(DriveInfo<double> &di1, const DriveInfo<int> &di2) {
	di1.FL = (di1.FL + abs(di2.FL)) / 2.0;
	di1.FR = (di1.FR + abs(di2.FR)) / 2.0;
	di1.RL = (di1.RL + abs(di2.RL)) / 2.0;
	di1.RR = (di1.RR + abs(di2.RR)) / 2.0;
}

inline void add(DriveInfo<double> &di1, const DriveInfo<double> &di2) {
	di1.FL = (di1.FL + fabs(di2.FL)) / 2.0;
	di1.FR = (di1.FR + fabs(di2.FR)) / 2.0;
	di1.RL = (di1.RL + fabs(di2.RL)) / 2.0;
	di1.RR = (di1.RR + fabs(di2.RR)) / 2.0;
}


inline double CalculateAverage(const DriveInfo<double> &data) {
	double sum = data.FL
			+ data.FR
			+ data.RL
			+ data.RR ;
	return sum / 4.0;
}


inline double CalculateAverage(const DriveInfo<int> &data) {
	double sum = data.FL
			+ data.FR
			+ data.RL
			+ data.RR ;
	return sum / 4.0;
}


void DmsProcessManager::SetRunning(bool _run) {
	if (running && !_run) {
		Reset();
	} else if (!running && _run) {
		currentPhase = kTestDriveMotors;
	}
	running = _run;
}


void DmsProcessManager::Run() {
	statusReporter->SetDmsMode(running);
	if (running) {
		switch(currentPhase) {
		case TestPhase::kStopped:
			Robot::driveBase->DMSDrive(0.0);
			Robot::driveBase->DMSSteer(0.0);
			break;
		case kTestDriveMotors:
			DoMotorTest();
			break;
		case kTestSteerMotors:
			DoSteerTest();
			break;
		}
	}
}


void DmsProcessManager::DoMotorTest() {
	const double currentTime = Timer::GetFPGATimestamp();
	if (startTime < 0) {
		std::cout << "Starting motor test\n";
		startTime = currentTime;
	}
	const double elapsedTime = (currentTime - startTime);

	if (elapsedTime < motorTestTime) {
		Robot::driveBase->DMSDrive(1.0);
		Robot::driveBase->DMSSteer(0.0);
		if (elapsedTime > initialIgnoreTime) {
			add(driveCurrent, Robot::driveBase->GetDriveCurrent());
			add(driveVelocity, Robot::driveBase->GetDMSDriveVelocity());
			loopCounter++;

			std::cout << "(DVel) FL: " << driveVelocity.FL
					  << " FR: " << driveVelocity.FR
					  << " RL: " << driveVelocity.RL
					  << " RR: " << driveVelocity.RR
					  << "\n";

			std::cout << "(DAmp) FL: " << driveCurrent.FL
					  << " FR: " << driveCurrent.FR
					  << " RL: " << driveCurrent.RL
					  << " RR: " << driveCurrent.RR
					  << "\n";

			const double velAvg = CalculateAverage(driveVelocity);
			const double ampAvg = CalculateAverage(driveCurrent);
			std::cout << "Vel Avg: " << velAvg << " | Amp Avg: " << ampAvg << "\n";
			DriveInfo<int> status;
			status.FL = CalculateStatus(driveVelocity.FL, velAvg, driveCurrent.FL, ampAvg);
			status.FR = CalculateStatus(driveVelocity.FR, velAvg, driveCurrent.FR, ampAvg);
			status.RL = CalculateStatus(driveVelocity.RL, velAvg, driveCurrent.RL, ampAvg);
			status.RR = CalculateStatus(driveVelocity.RR, velAvg, driveCurrent.RR, ampAvg);

			statusReporter->SetDriveStatus(status);

			std::cout << "Drive FL: " << status.FL
					  << " FR: " << status.FR
					  << " RL: " << status.RL
					  << " RR: " << status.RR
					  << "\n";
		}
	} else {
		startTime = -1;
		currentPhase = kTestSteerMotors;
		std::cout << "******************** STARTING STEER PHASE ********************\n";
	}
}


void DmsProcessManager::DoSteerTest() {
	const double currentTime = Timer::GetFPGATimestamp();
	if (startTime < 0) {
		startTime = currentTime;
	}
	const double elapsedTime = (currentTime - startTime);

	if (elapsedTime < motorTestTime) {
		Robot::driveBase->DMSSteer(1.0);
		Robot::driveBase->DMSDrive(0.0);
		if (elapsedTime > initialIgnoreTime) {
			add(steerCurrent, Robot::driveBase->GetSteerCurrent());
			add(steerVelocity, Robot::driveBase->GetDMSSteerVelocity());
			loopCounter++;

			std::cout << "(SVel) FL: " << steerVelocity.FL
					  << " FR: " << steerVelocity.FR
					  << " RL: " << steerVelocity.RL
					  << " RR: " << steerVelocity.RR
					  << "\n";

			std::cout << "(SAmp) FL: " << steerCurrent.FL
					  << " FR: " << steerCurrent.FR
					  << " RL: " << steerCurrent.RL
					  << " RR: " << steerCurrent.RR
					  << "\n";

			const double velAvg = CalculateAverage(steerVelocity);
			const double ampAvg = CalculateAverage(steerCurrent);
			std::cout << "Vel Avg: " << velAvg << " | Amp Avg: " << ampAvg << "\n";
			DriveInfo<int> status;
			status.FL = CalculateStatus(steerVelocity.FL, velAvg, steerCurrent.FL, ampAvg);
			status.FR = CalculateStatus(steerVelocity.FR, velAvg, steerCurrent.FR, ampAvg);
			status.RL = CalculateStatus(steerVelocity.RL, velAvg, steerCurrent.RL, ampAvg);
			status.RR = CalculateStatus(steerVelocity.RR, velAvg, steerCurrent.RR, ampAvg);

			statusReporter->SetSteerStatus(status);

			std::cout << "Steer FL: " << status.FL
					  << " FR: " << status.FR
					  << " RL: " << status.RL
					  << " RR: " << status.RR
					  << "\n";
		}
	} else {
		std::cout << "******************** FINISHED STEER PHASE ********************\n";
		currentPhase = kStopped;

		frc::SmartDashboard::PutNumber("FL V", driveVelocity.FL);
		frc::SmartDashboard::PutNumber("FR V", driveVelocity.FR);
		frc::SmartDashboard::PutNumber("RL V", driveVelocity.RL);
		frc::SmartDashboard::PutNumber("RR V", driveVelocity.RR);

		frc::SmartDashboard::PutNumber("FL A", driveCurrent.FL);
		frc::SmartDashboard::PutNumber("FR A", driveCurrent.FR);
		frc::SmartDashboard::PutNumber("RL A", driveCurrent.RL);
		frc::SmartDashboard::PutNumber("RR A", driveCurrent.RR);


		frc::SmartDashboard::PutNumber("St FL V", steerVelocity.FL);
		frc::SmartDashboard::PutNumber("St FR V", steerVelocity.FR);
		frc::SmartDashboard::PutNumber("St RL V", steerVelocity.RL);
		frc::SmartDashboard::PutNumber("St RR V", steerVelocity.RR);

		frc::SmartDashboard::PutNumber("St FL A", steerCurrent.FL);
		frc::SmartDashboard::PutNumber("St FR A", steerCurrent.FR);
		frc::SmartDashboard::PutNumber("St RL A", steerCurrent.RL);
		frc::SmartDashboard::PutNumber("St RR A", steerCurrent.RR);

	}
}



int DmsProcessManager::CalculateStatus(const double enc, const double encAvg, const double amp, const double ampAvg) {
	if (amp == 0) {
		return 4;	// No Amps
	}
	if (enc == 0) {
		return 5;	// No encoder counts
	}
	bool encOutside = (enc / encAvg) < encAvgThreshold;
	bool ampOutside = (amp / ampAvg) < ampAvgThreshold;
	if (!encOutside && !ampOutside) {
		return 1;	// Good
	} else if (!encOutside && ampOutside) {
		return 2;	// Amp outside, enc ok
	} else if (encOutside && !ampOutside) {
		return 3;	// Enc outside of value, amp is ok
	} else {
		// Both enc & amp outside of ranges
		return 2; 	// both outside, show amp
	}
}


void DmsProcessManager::Reset() {
	currentPhase = kStopped;
	startTime = -1;
	loopCounter = 0;
	driveCurrent = DriveInfo<double>(0.0);
	driveVelocity = DriveInfo<double>(0.0);
	steerCurrent = DriveInfo<double>(0.0);
	steerVelocity = DriveInfo<double>(0.0);
}
