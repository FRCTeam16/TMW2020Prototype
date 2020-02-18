#pragma once

#include "Robot.h"
#include "Autonomous/Step.h"

class EnableVisionTracking : public Step {
public:
    explicit EnableVisionTracking(bool enable)
    : enable(enable) {
    }

    bool Run(std::shared_ptr<World> world) override {
        std::cout << "EnableVisionTracking(" << enable << ")\n";
        Robot::turret->SetShooterEnabled(enable);
        if (enable) {
            Robot::turret->GetTurretRotation().EnableVisionTracking();
        } else {
            Robot::turret->GetTurretRotation().DisableVisionTracking();
        }
        return true;
    }
private:
    bool enable;
};