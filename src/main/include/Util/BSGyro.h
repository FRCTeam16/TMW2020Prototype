#pragma once

#include <frc/PIDSource.h>
#include <ctre/Phoenix.h>

using namespace frc;

class BSGyro : public frc::PIDSource {
private:
	std::unique_ptr<PigeonIMU> pigeon;
	float GetOffset();
	float offset = 0.0;
public:
    explicit BSGyro(WPI_TalonSRX *talon);

	explicit BSGyro(int canId);
	virtual ~BSGyro();
	double PIDGet();
	PigeonIMU* GetPigeon();
	void SetOffset(float offset);

    float GetYaw();
    void ZeroYaw();

	double ReadYaw();

};
