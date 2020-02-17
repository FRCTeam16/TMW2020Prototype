#pragma once

#include "Robot.h"
#include "Subsystems/Turret/Turret.h"
#include "Subsystems/FeederArm/FeederArm.h"

class ShortShotPose {
public:
    ShortShotPose(std::shared_ptr<Turret> turret, std::shared_ptr<FeederArm> feederArm) : turret(turret), feederArm(feederArm) {}

    void Run() {
        turret->SetLidToShortShot();
        turret->SetShooterEnabled(true);
        // turret->RotateTurret(Turret::TurretPosition::kFront);
        // arm up?
    }

private:
    const std::shared_ptr<Turret> turret;
    const std::shared_ptr<FeederArm> feederArm;
};