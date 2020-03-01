#pragma once

#include "Autonomous/Step.h"
#include "Robot.h"

class SelectShootingProfile : public Step {
public:
    explicit SelectShootingProfile(ShootingProfile profile)        
    : profile(profile)
    {
    }

    bool Run(std::shared_ptr<World> world) override
    {
        std::cout << "SelectShootingProfile ran\n";
        Robot::turret->SetShootingProfile(profile);
        return true;
    }

private:
    const ShootingProfile profile;

};