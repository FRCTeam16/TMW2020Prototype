#pragma once

#include "Autonomous/Step.h"
#include "RobotMap.h"
#include "Gyro/CollisionDetector.h"

class DriveToBump : public Step {
public:
	DriveToBump(double _angle, double _ySpeed, double _xSpeed, double _maxDriveTime,
			    double _delayCheckTime, double _gThreshold) :
		angle(_angle),
		ySpeed(_ySpeed),
		xSpeed(_xSpeed),
		maxTimeToDrive(_maxDriveTime),
		delayCheckTime(_delayCheckTime),
		collisionDetector(RobotMap::gyro->GetCollisionDetector(_gThreshold)) {}

	virtual ~DriveToBump() {}
	bool Run(std::shared_ptr<World> world) override;

	void SetRampTime(double _time) { rampTime = _time; }

private:
     const double angle;
     const double ySpeed;
     const double xSpeed;
     const double maxTimeToDrive;
     const double delayCheckTime;	// delay before checking for collision
     const std::unique_ptr<CollisionDetector> collisionDetector;

     double startTime = -1;
     bool collisionDetected = false;

     double rampTime = -1;
};
