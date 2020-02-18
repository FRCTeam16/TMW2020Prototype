#pragma once

#include "Robot.h"
#include "Subsystems/Turret/Turret.h"
#include "Subsystems/FeederArm/FeederArm.h"


class LongShotPose {
public:
    LongShotPose(std::shared_ptr<Turret> turret, std::shared_ptr<FeederArm> feederArm) : turret(turret), feederArm(feederArm) {}

    void Run() {
        turret->SetLidToLongShot();
        turret->SetTurretPosition(Turret::Position::kBack);
        feederArm->DebugSetPoint(0);
    }

private:
    const std::shared_ptr<Turret> turret;
    const std::shared_ptr<FeederArm> feederArm;
};