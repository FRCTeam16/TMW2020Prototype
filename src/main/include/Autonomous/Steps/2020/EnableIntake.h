#pragma once

#include "Robot.h"
#include "Autonomous/Step.h"

class EnableIntake : public Step {
public:
    explicit EnableIntake(bool enableIntake)
    : enableIntake(enableIntake) {
    }

    bool Run(std::shared_ptr<World> world) override {
        if (enableIntake) {
            Robot::feederArm->StartIntake(reversed);
        } else {
            Robot::feederArm->StopIntake();
        }
        return true;
    }
private:
    bool enableIntake;
    bool reversed = false;

};