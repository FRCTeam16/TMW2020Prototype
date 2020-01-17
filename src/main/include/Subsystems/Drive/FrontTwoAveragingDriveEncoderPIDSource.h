#pragma once

#include "Subsystems/Drive/DriveInfo.h"
#include "Subsystems/Drive/SwerveWheel.h"
#include <frc/PIDSource.h>
#include <memory>

class FrontTwoAveragingDriveEncoderPIDSource : public frc::PIDSource {
public:
	explicit FrontTwoAveragingDriveEncoderPIDSource(DriveInfo<std::shared_ptr<SwerveWheel>> _wheels);
	virtual ~FrontTwoAveragingDriveEncoderPIDSource();
	virtual double PIDGet();
	void SetInitialEncoderValue();
	void SetShowDebug(bool _showDebug);
private:
	DriveInfo<std::shared_ptr<SwerveWheel>> wheels;
	DriveInfo<double> initialEncoderValue;
	double CalculateAverage(const DriveInfo<double> &error, const DriveInfo<bool> &motorEnabled);
	bool showDebug = false;
};

