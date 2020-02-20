#include "Poses/ShotPoses.h"

void ShortShotPose::Run(bool startShooter) {
    turret->SetLidToShortShot();
    turret->SetShooterEnabled(startShooter);
    turret->GetTurretRotation().SetTurretPosition(TurretRotation::Position::kFront);
    if (!startShooter) {
        feederArm->SetArmPosition(FeederArm::Position::kZero);
    }
}


void LongShotPose::Run(bool startShooter) {
    turret->SetLidToLongShot();
    turret->GetTurretRotation().SetTurretPosition(TurretRotation::Position::kBack);
    feederArm->SetArmPosition(FeederArm::Position::kZero);
}