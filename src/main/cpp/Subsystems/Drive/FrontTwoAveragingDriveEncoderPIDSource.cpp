#include <iostream>
#include "Subsystems/Drive/FrontTwoAveragingDriveEncoderPIDSource.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

FrontTwoAveragingDriveEncoderPIDSource::FrontTwoAveragingDriveEncoderPIDSource(DriveInfo<std::shared_ptr<SwerveWheel>> _wheels)
: wheels(_wheels){
	frc::PIDSource::SetPIDSourceType(frc::PIDSourceType::kDisplacement);
}

FrontTwoAveragingDriveEncoderPIDSource::~FrontTwoAveragingDriveEncoderPIDSource() = default;

void FrontTwoAveragingDriveEncoderPIDSource::SetInitialEncoderValue() {
	initialEncoderValue.FL = wheels.FL->GetDriveEncoderPosition();
	initialEncoderValue.FR = wheels.FR->GetDriveEncoderPosition();
	initialEncoderValue.RL = wheels.RL->GetDriveEncoderPosition();
	initialEncoderValue.RR = wheels.RR->GetDriveEncoderPosition();
}

double FrontTwoAveragingDriveEncoderPIDSource::PIDGet() {
	// Calculate current error
	DriveInfo<double> error;
	DriveInfo<bool> motorEnabled {true};

	error.FL = fabs(wheels.FL->GetDriveEncoderPosition() - initialEncoderValue.FL);
	error.FR = fabs(wheels.FR->GetDriveEncoderPosition() - initialEncoderValue.FR);

	SmartDashboard::PutNumber("FLEncoderTravel", error.FL);
	SmartDashboard::PutNumber("FREncoderTravel", error.FR);
	SmartDashboard::PutNumber("RLEncoderTravel", (wheels.RL->GetDriveEncoderPosition() - initialEncoderValue.RL));
	SmartDashboard::PutNumber("RREncoderTravel", (wheels.RR->GetDriveEncoderPosition() - initialEncoderValue.RR));

	if (showDebug) {
		std::cout << "AvgDrivePID error.FL " << error.FL << "\n";
		std::cout << "AvgDrivePID error.FR " << error.FR << "\n";
	}

	// Calculate initial working average
	const double initialAverage = CalculateAverage(error, motorEnabled);
	if (showDebug) { std::cout << "AvgDrivePID initialAverage = " << initialAverage << "\n"; }


	// Vote for encoder inclusion
	/*const double threshold = 0.5;
	int enabledCount = 4;
	if (((error.FL / initialAverage) < (1 - threshold)) || ((error.FL/initialAverage) > (threshold + 1)) || (error.FL < 10 && initialAverage > 100)) {
		motorEnabled.FL = false;
		enabledCount--;
	}
	if (((error.FR / initialAverage) < (1- threshold)) || ((error.FR/initialAverage) > (threshold + 1)) || (error.FR < 10 && initialAverage > 100)) {
		motorEnabled.FR = false;
		enabledCount--;
	}
	if (((error.RL / initialAverage) < (1- threshold)) || ((error.RL/initialAverage) > (threshold + 1)) || (error.RL < 10 && initialAverage > 100)) {
		motorEnabled.RL = false;
		enabledCount--;
	}
	if (((error.RR / initialAverage) < (1- threshold)) || ((error.RR/initialAverage) > (threshold + 1)) || (error.RR < 10 && initialAverage > 100)) {
		motorEnabled.RR = false;
		enabledCount--;
	}*/

	if (showDebug) {
		std::cout << "AvgDrivePID motorEnabled.FL " << motorEnabled.FL << "\n";
		std::cout << "AvgDrivePID motorEnabled.FR " << motorEnabled.FR << "\n";
	}
	// If we removed some encoders for being too out of sync, recalculate average
	double returnedError = initialAverage;
//	if (initialAverage > 300 && (enabledCount > 1 && enabledCount < 4)) {
//		returnedError =  CalculateAverage(error, motorEnabled);
//	}
	if (showDebug) { std::cout << "AvgDrivePID returnedError = " << returnedError << "\n"; }

	SmartDashboard::PutBoolean("PID.FL", motorEnabled.FL);
	SmartDashboard::PutBoolean("PID.FR", motorEnabled.FR);
	SmartDashboard::PutBoolean("PID.RL", motorEnabled.RL);
	SmartDashboard::PutBoolean("PID.RR", motorEnabled.RR);

	return returnedError;
}

double FrontTwoAveragingDriveEncoderPIDSource::CalculateAverage(const DriveInfo<double> &error, const DriveInfo<bool> &motorEnabled) {
	// Calculate average error
	double sum = 0.0;
	int count = 0;
	if (motorEnabled.FL) { sum += error.FL; count++; }
	if (motorEnabled.FR) { sum += error.FR; count++; }
	const double average = sum / count;
	return average;
}

void FrontTwoAveragingDriveEncoderPIDSource::SetShowDebug(bool _showDebug) {
	showDebug = _showDebug;
}



