#pragma once

#include <memory>
#include "Subsystems/FeederArm/FeederArm.h"
#include "Subsystems/Turret/Turret.h"

class ShotPose {
public:    
    ShotPose(std::shared_ptr<Turret> turret, std::shared_ptr<FeederArm> feederArm) : turret(turret), feederArm(feederArm) {}
    virtual void Run(bool startShooter = true) = 0;
protected:
    std::shared_ptr<Turret> turret;
    std::shared_ptr<FeederArm> feederArm;
};


class ShortShotPose : public ShotPose {
public:
    ShortShotPose(std::shared_ptr<Turret> turret, std::shared_ptr<FeederArm> feederArm) : ShotPose{turret, feederArm} {}
    void Run(bool startShooter = true) override;
};


class LongShotPose : public ShotPose {
public:
    LongShotPose(std::shared_ptr<Turret> turret, std::shared_ptr<FeederArm> feederArm) : ShotPose{turret, feederArm} {}
    void Run(bool startShooter = true) override;
};

    