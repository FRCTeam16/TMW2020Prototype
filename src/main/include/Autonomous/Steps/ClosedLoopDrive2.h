
#ifndef SRC_AUTONOMOUS_STEPS_CLOSEDLOOPDRIVE2_H_
#define SRC_AUTONOMOUS_STEPS_CLOSEDLOOPDRIVE2_H_

#include "Autonomous/Step.h"
#include "Autonomous/DriveUnit.h"
#include "Util/CollisionDetector.h"
#include "Util/DistanceControl.h"
#include "RobotMap.h"

class ClosedLoopDrive2: public Step {
public:
	ClosedLoopDrive2(double _angle, double _speed, double _x, double _y,
					double _threshold, DriveUnit::Units _units,
					double _timeout,
					double _rampUp, double _rampDown) :
		angle(_angle),
		speed(_speed),
		XtargetDistance(_x),
		YtargetDistance(_y),
		distanceThreshold(_threshold),
		units(_units),
		timeoutCommand(_timeout),
		rampUp(_rampUp),
		rampDown(_rampDown),
		collisionDetector(CollisionDetector(RobotMap::gyro, 2.0))
	{
	}

	virtual ~ClosedLoopDrive2();
	bool Run(std::shared_ptr<World> world) override;
	void setUseCurrentAngle();

	// Custom behaviors
	void SetHaltOnIntakePickup(bool _halt) { haltOnIntakePickup = _halt; }
	void UsePickupDistance(bool _invertDistance = false) { usePickupDistance = true; invertPickupDistance = _invertDistance; }
	void UseDriveDistanceOvershoot(bool _invertDistance = false) { useDriveDistanceOvershoot = true; invertOvershootDistance = _invertDistance; }
	void SetHardStopsContinueFromStep(bool _stay) { hardStopsContinueFromStep = _stay; }
	void SetRampUpMin(double _min) { rampUpMin = _min; }
	void SetRampDownMin(double _min) { rampDownMin = _min; }

	// Drive distance control uses an ultrasonic to control X component of drive
	void EnableDistanceControl(double _target, bool _invert, double _ignoreTime = 1.5);
	DistanceControl* GetDistanceControl() const;


private:
	void StoreDistance(World* world);

	const double angle;
	const double speed;
	double XtargetDistance;
	double YtargetDistance;
	const double distanceThreshold;     // -1 value simply checks for passing the setpoint
	const DriveUnit::Units units;
	const double timeoutCommand = 10;
	const double rampUp;	// ramp up time length, -1 to disable
	const double rampDown;	// ramp down position threshold, -1 to disable

	double rampUpMin = 0.10;	// ramp up min speed
	double rampDownMin = 0.10;	// ramp down min speed

	CollisionDetector collisionDetector;
	const bool reverse = false;
	const bool useGyro = true;
	const int thresholdCounterTarget = 1;


	double startTime = -1;
	bool useCurrentAngle = false;
	int thresholdCounter = 0;
	double targetSetpoint = 0;
	double startEncoderPosition = 0;

	bool haltOnIntakePickup = false;	// TODO: replace with strategy

	bool usePickupDistance = false;			// use tracked X movement
	bool invertPickupDistance = false;		// whether to invert the direction of the X pickup distance

	bool useDriveDistanceOvershoot = false;	// whether to incorporate overshoot from World into Y distance
	bool invertOvershootDistance = false;	// whether to invert the distance of the overshoot when adding to total Y

	bool hardStopsContinueFromStep = true;	// when collisions or timeouts occur, whether to stay on step or continue

	const bool debug = true;

	std::unique_ptr<DistanceControl> distanceControl;	// optional distance control
	double distanceControlIgnoreTime;

};

#endif /* SRC_AUTONOMOUS_STEPS_CLOSEDLOOPDRIVE2_H_ */
