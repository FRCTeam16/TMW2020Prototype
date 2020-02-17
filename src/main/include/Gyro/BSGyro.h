#pragma once

#include <frc/PIDSource.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Gyro/CollisionDetector.h"


class BSGyro : public frc::PIDSource {
public:
	virtual ~BSGyro() {};
	double PIDGet() { return GetYaw(); }

	void Instrument()
	{
		frc::SmartDashboard::PutNumber("Yaw", GetYaw());
		frc::SmartDashboard::PutNumber("YawOffset", GetOffset());
		frc::SmartDashboard::PutNumber("RawYaw", ReadRawYaw());
		GyroSpecificInstrument();
	}

	void SetOffset(float _offset) { offset = _offset; }
	double GetOffset() { return offset; }

    double GetYaw()
	{
		const double rawYaw = ReadRawYaw();
        if (rawYaw > 180.0) { return rawYaw - 360.0; }
        else if (rawYaw < -180.0) { return rawYaw + 360.0; }
        else { return rawYaw; }
	}

    virtual void ZeroYaw() = 0;
	virtual std::unique_ptr<CollisionDetector> GetCollisionDetector(double gThreshold) = 0;
private:
	float offset = 0.0;
	virtual double ReadRawYaw() = 0;
	virtual void GyroSpecificInstrument() {};
};

