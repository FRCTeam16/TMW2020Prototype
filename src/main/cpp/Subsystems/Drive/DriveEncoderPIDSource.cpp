#include "Subsystems/Drive/DriveEncoderPIDSource.h"

DriveEncoderPIDSource::DriveEncoderPIDSource(std::shared_ptr<WPI_TalonSRX> _motor, int *_inverted) {
	motor = _motor;
	inverted = _inverted;
	PIDSource::SetPIDSourceType(frc::PIDSourceType::kDisplacement);
}

DriveEncoderPIDSource::~DriveEncoderPIDSource() = default;

void DriveEncoderPIDSource::SetInitialEncoderValue() {
	initialEncoderValue = motor->GetSelectedSensorPosition(0);
}

double DriveEncoderPIDSource::PIDGet() {
	return fabs(motor->GetSelectedSensorPosition(0) - initialEncoderValue);
}

