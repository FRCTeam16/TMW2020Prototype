#include "Autonomous/Steps/VisionControlledDrive.h"
#include "Robot.h"
#include "Subsystems/Drive/CrabInfo.h"
#include "Autonomous/Steps/ClosedLoopDrive2.h"
#include "Autonomous/Steps/TimedDrive.h"

VisionControlledDrive::VisionControlledDrive(
    double _angle, double _speed, double _x, double _y,
    double _threshold, DriveUnit::Units _units,
    double _timeout,
    double _rampUp, double _rampDown) {

    drive.reset(new ClosedLoopDrive2(
        _angle, _speed, _x, _y, _threshold, _units, _timeout, _rampUp, _rampDown
    ));
}

VisionControlledDrive::VisionControlledDrive(double _angle, double y, double x, double driveTime, bool _useTwist) {
    drive.reset(new TimedDrive(_angle, y, x, driveTime, _useTwist));
}

bool VisionControlledDrive::Run(std::shared_ptr<World> world) {
    const bool driveFinished = drive->Run(world);
    auto vision = Robot::visionSystem->GetLastVisionInfo();
    auto driveCrab = drive->GetCrabInfo();
    double xspeed = driveCrab->xspeed;

    std::cout << "VisionControlledDrive: tgt? " << vision->hasTarget
                  << " | xspeed: " << vision->xSpeed
                  << " | ta: " << vision->targetArea
                  << "\n";

    if (vision->hasTarget) {
        // std::cout << "VisionControlleDrive: tgt? " << hasTarget
        //           << " | xspeed: " << vision->xSpeed
        //           << " | ta: " << vision->targetArea
        //           << "\n";
        xspeed = vision->xSpeed;
    }

    crab->Update(driveCrab->twist, driveCrab->yspeed, xspeed, driveCrab->gyro);
    return driveFinished;   // FIXME: || in vision position?
}