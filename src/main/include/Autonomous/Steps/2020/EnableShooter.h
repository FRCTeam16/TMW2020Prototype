#pragma once

#include "Robot.h"
#include "Autonomous/Step.h"

class EnableShooter : public Step {
public:
    explicit EnableShooter(bool enableShooter)
    : enableShooter(enableShooter) {
    }

    bool Run(std::shared_ptr<World> world) override {
        Robot::turret->SetShooterEnabled(enableShooter);
        return true;
    }
private:
    bool enableShooter;
};