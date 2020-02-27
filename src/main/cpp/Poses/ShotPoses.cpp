#include "Poses/ShotPoses.h"
#include "Robot.h"

void ShortShotPose::Run(bool startShooter) {
    Robot::turret->SetShootingProfile(ShootingProfile::kShort);
    Robot::turret->SetLidToShortShot();
    Robot::turret->SetShooterEnabled(startShooter);
    Robot::turret->GetTurretRotation().SetTurretPosition(TurretRotation::Position::kFront);
    if (!startShooter) {
        Robot::feederArm->SetArmPosition(FeederArm::Position::kZero);
    }
    Robot::feederArm->SetArmPosition(FeederArm::Position::kPlayerStation);
}

void MediumShotPose::Run(bool startShooter) {
    Robot::turret->SetShootingProfile(ShootingProfile::kMedium);
    Robot::turret->SetLidToLongShot();
    Robot::turret->GetTurretRotation().SetTurretPosition(TurretRotation::Position::kBack);
    // Robot::feederArm->SetArmPosition(FeederArm::Position::kZero);
    Robot::visionSystem->GetLimelight()->SelectPipeline(0);
    Robot::visionSystem->EnableVisionTracking();
}

void LongShotPose::Run(bool startShooter) {
    Robot::turret->SetShootingProfile(ShootingProfile::kLong);
    Robot::turret->SetLidToLongShot();
    Robot::turret->GetTurretRotation().SetTurretPosition(TurretRotation::Position::kBack);
    // Robot::feederArm->SetArmPosition(FeederArm::Position::kZero);
    Robot::visionSystem->GetLimelight()->SelectPipeline(1);
    Robot::visionSystem->EnableVisionTracking();
}

void TrenchShotPose::Run(bool shartShooter) {
    Robot::turret->SetShootingProfile(ShootingProfile::kMedium);
    Robot::turret->SetLidToLongShot();
    Robot::turret->GetTurretRotation().SetTurretPosition(TurretRotation::Position::kGoalWallShot);
    // Robot::feederArm->SetArmPosition(FeederArm::Position::kZero);
    Robot::visionSystem->DisableVisionTracking();
}