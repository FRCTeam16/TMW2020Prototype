#pragma once

#include <ctre/Phoenix.h>
#include "Subsystems/Drive/DriveInfo.h"
#include <frc/PIDSource.h>


class AveragingDriveEncoderPIDSource : public frc::PIDSource {
public:
    explicit AveragingDriveEncoderPIDSource(DriveInfo<std::shared_ptr<WPI_TalonSRX>> _motor);

	~AveragingDriveEncoderPIDSource() override;

	double PIDGet() override;
	void SetInitialEncoderValue();
	void SetShowDebug(bool _showDebug);
private:
	DriveInfo<std::shared_ptr<WPI_TalonSRX>> motor;
	DriveInfo<double> initialEncoderValue;
	double CalculateAverage(const DriveInfo<double> &error, const DriveInfo<bool> &motorEnabled);
	bool showDebug = false;
};
