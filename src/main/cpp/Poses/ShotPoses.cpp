#include "Poses/ShotPoses.h"

void ShortShotPose::Run(bool startShooter) {
    turret->SetLidToShortShot();
    turret->SetShooterEnabled(startShooter);
    turret->SetTurretPosition(Turret::Position::kFront);
    if (!startShooter) {
        feederArm->DebugSetPoint(0);
    }
}


void LongShotPose::Run(bool startShooter) {
    turret->SetLidToLongShot();
    turret->SetTurretPosition(Turret::Position::kBack);
    feederArm->DebugSetPoint(0);
}