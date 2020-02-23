#pragma once

#include <memory>
#include "Subsystems/FeederArm/FeederArm.h"
#include "Subsystems/Turret/Turret.h"

class ShotPose {
public:    
    virtual void Run(bool startShooter = true) = 0;
};


class ShortShotPose : public ShotPose {
public:
    void Run(bool startShooter = true) override;
};

class MediumShotPose : public ShotPose {
public:
    void Run(bool startShooter = true) override;
};

class LongShotPose : public ShotPose {
public:
    void Run(bool startShooter = true) override;
};

class TrenchShotPose : public ShotPose {
public:
    void Run(bool startShooter = true) override;
};

    