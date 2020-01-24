#include "Robot.h"
#include "RobotMap.h"
#include "Autonomous/Steps/ClosedLoopDrive2.h"
#include "Util/RampUtil.h"


ClosedLoopDrive2::~ClosedLoopDrive2() {
}

void ClosedLoopDrive2::setUseCurrentAngle() {
	useCurrentAngle = true;
}

bool ClosedLoopDrive2::Run(std::shared_ptr<World> world) {
	bool doExit = false;
	if (startTime < 0) {
		startTime = world->GetClock();

		/**
		 * If we are using pickup distance, we will use the X drive distance stored on the World object by
		 * a previous drive step as the additoinal amount to add to our configured X drive distance
		 */
		if (usePickupDistance) {
			if (debug) std::cout << "ClosedLoopDrive2 using pickup distance: original = " << XtargetDistance;
			 double driveDistance = DriveUnit::ToInches(world->GetDriveDistance(), DriveUnit::kPulses);
			if (invertPickupDistance) {
				driveDistance *= -1.0;
			}
			XtargetDistance += driveDistance;
			if (debug) std::cout << " new = " << XtargetDistance << "\n";
		} else if (useDriveDistanceOvershoot) {
			double overshoot = world->GetDriveDistanceOvershoot();
			double driveDistance = DriveUnit::ToInches(overshoot, DriveUnit::kPulses);
			if (invertOvershootDistance) {
				driveDistance *= -1.0;
			}
			std::cout << "ClosedLoopDrive2 using overshoot distance: original Y = " << YtargetDistance
					<< " | overshoot = " << overshoot
					<< " | drive To Add = " << driveDistance;


			YtargetDistance += driveDistance;
			std::cout << " new Y = " << YtargetDistance << "\n";
		}


		const double hypotenuse = sqrt(XtargetDistance * XtargetDistance + YtargetDistance * YtargetDistance);
		targetSetpoint = DriveUnit::ToPulses(hypotenuse, units);
		
		const double targetAngle = (!useCurrentAngle) ? angle : RobotMap::gyro->GetYaw();
		Robot::driveBase->SetTargetAngle(targetAngle);

		if (debug) std::cout << "Setting Target Drive Distance:" << targetSetpoint << "| Speed:" << speed << "\n";
		Robot::driveBase->SetTargetDriveDistance(targetSetpoint, speed);
		Robot::driveBase->UseClosedLoopDrive();

		startEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
	}

	const double currentEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
//	const double currentError = targetSetpoint - currentEncoderPosition;
	const double elapsedTimeSecs = world->GetClock() - startTime;
	const double currentPIDOutput = Robot::driveBase->GetDriveControlOutput();
	double thresholdPassInverter = 1;	// when driving past the threshold, we will invert this value to avoid driving in a negative directoin

	if (debug) {
		SmartDashboard::PutNumber("PIDController Output", currentPIDOutput);
		std::cout << "XYPIDControlledDrive target setpoint          = " << targetSetpoint << "\n";
		std::cout << "XYPIDControlledDrive Current Encoder Position = " << currentEncoderPosition << "\n";
	//	std::cout << "XYPIDControlledDrive Current Error            = " << currentError << "\n";
	//	std::cout << "XYPIDControlledDrive Current Threshold        = " << DriveUnit::ToPulses(distanceThreshold, units) << "\n";
	//	std::cout << "XYPIDControlledDrive PID Output:              " << currentPIDOutput << "\n";


	//	std::cout << "Start Time  : " << startTime << "\n";
		std::cout << "Elapsed Time: " << elapsedTimeSecs << "\n";
	//	std::cout << "Clock: " << world->GetClock() << "\n";
	}

	SmartDashboard::PutNumber("DriveControl P", Robot::driveBase->GetDriveControlP());


	// Store our current information to the world
	StoreDistance(world.get());


	/**
	 * Distance threshold checks for halting
	 */
	if (distanceThreshold == -1) {
		if (currentEncoderPosition > targetSetpoint) {
			std::cout << "!!!Current encoder passed target\n";
//			thresholdPassInverter = -1.0;	// FIXME: Weirdness
//			doExit = true;
			return true;
		}
	} else if (currentEncoderPosition > targetSetpoint) {
		if (thresholdCounter++ >= thresholdCounterTarget) {
			std::cout << "!!!Position reached in " << elapsedTimeSecs << "\n";
			crab->Stop();
			return true;
		}
	} else {
		thresholdCounter = 0;
	}


	if (elapsedTimeSecs > timeoutCommand) {
		std::cerr << "**** EMERGENCY EXIT OF STEP DUE TO TIMEOUT ***\n";
		crab->Stop();
		return hardStopsContinueFromStep;
	} else if (elapsedTimeSecs > 1.0 &&  collisionDetector.Detect()) {
		std::cerr << "**** EMERGENCY HALT DUE TO COLLISION ****\n";
		crab->Stop();
		return hardStopsContinueFromStep;
	// } else if (haltOnIntakePickup && Robot::intake->IsPickupTriggered()) {
	// 	std::cout << "!!! Detected Halt on Intake Pickup! \n";
	// 	return true;
	} else {
		const double crabSpeed = speed * ((reverse) ? -1.0 : 1.0);

		/**
		 * Ramp Control
		 */
		double profiledSpeed = crabSpeed * thresholdPassInverter;
		if (rampUp > 0) {
			profiledSpeed = RampUtil::RampUp(profiledSpeed, elapsedTimeSecs, rampUp, rampUpMin);
		}
		if (rampDown > 0) {
			profiledSpeed =  RampUtil::RampDown(profiledSpeed, currentEncoderPosition, targetSetpoint, DriveUnit::ToPulses(rampDown, units), rampDownMin);
		}

		const double angleRadians = atan2(XtargetDistance, YtargetDistance);
		const double yspeed = profiledSpeed * cos(angleRadians);
		double xspeed = profiledSpeed * sin(angleRadians);

		if (distanceControl && (elapsedTimeSecs > distanceControlIgnoreTime)) {
			xspeed += distanceControl->Get();
		}

		const double twistOutput = Robot::driveBase->GetTwistControlOutput();

		crab->Update(
				(float) twistOutput,
				(float) yspeed,
				(float) xspeed,
				useGyro);
		return doExit;
	}
}

void ClosedLoopDrive2::StoreDistance(World* world) {
	double endEncoderPosition = Robot::driveBase->GetDriveControlEncoderPosition();
	double overshoot = endEncoderPosition - targetSetpoint;
	world->SetDriveDistance(endEncoderPosition - startEncoderPosition);
	world->SetDriveDistanceOvershoot(overshoot);

}

void ClosedLoopDrive2::EnableDistanceControl(double _target, bool _invert, double _ignoreTime) {
	// distanceControl.reset(new DistanceControl(_target));
	// distanceControl->SetInvert(_invert);
	// distanceControlIgnoreTime = _ignoreTime;
	throw logic_error("ultrasonic distance control not enabled");
}

DistanceControl* ClosedLoopDrive2::GetDistanceControl() const {
	return distanceControl.get();
}

