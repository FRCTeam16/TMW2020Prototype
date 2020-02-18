#pragma once

#include "Robot.h"
#include "Subsystems/Turret/Turret.h"
#include "Subsystems/FeederArm/FeederArm.h"

class ShortShotPose {
public:
    ShortShotPose(std::shared_ptr<Turret> turret, std::shared_ptr<FeederArm> feederArm) : turret(turret), feederArm(feederArm) {}

    void Run(bool startShooter = true) {
        turret->SetLidToShortShot();
        turret->SetShooterEnabled(startShooter);
        turret->SetTurretPosition(Turret::Position::kFront);
        if (!startShooter) {
            feederArm->DebugSetPoint(0);
        }
    }

private:
    const std::shared_ptr<Turret> turret;
    const std::shared_ptr<FeederArm> feederArm;
};