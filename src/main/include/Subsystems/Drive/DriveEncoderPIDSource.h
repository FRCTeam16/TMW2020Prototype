/*
 * DriveEncoderPIDSource.h
 *
 *  Created on: Jan 28, 2017
 *      Author: User
 */

#ifndef SRC_SUBSYSTEMS_DRIVEENCODERPIDSOURCE_H_
#define SRC_SUBSYSTEMS_DRIVEENCODERPIDSOURCE_H_

#include "frc/PIDSource.h"
#include "ctre/Phoenix.h"

using namespace frc;

class DriveEncoderPIDSource : public frc::PIDSource {
public:
	DriveEncoderPIDSource(std::shared_ptr<WPI_TalonSRX> _motor, int *_inverted);

	~DriveEncoderPIDSource() override;
	virtual double PIDGet();
	void SetInitialEncoderValue();
private:
	std::shared_ptr<WPI_TalonSRX> motor;
	int *inverted;
	double initialEncoderValue = 0;
};

#endif /* SRC_SUBSYSTEMS_DRIVEENCODERPIDSOURCE_H_ */
