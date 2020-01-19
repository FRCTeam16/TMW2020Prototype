#pragma once

#include "Subsystems/SubsystemManager.h"
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotMap.h"
#include <iostream>

class Intake : public SubsystemManager {
public:

    void Init() override {
        frc::SmartDashboard::PutNumber("IntakeSpeed", 0.8);
    }

    void Run() override {
        double speed = 0.0;
        if (enabled) {
            speed = frc::SmartDashboard::GetNumber("IntakeSpeed", 0.8);
        }
        intakeMotor->Set(speed);
    }

    void ToggleEnabled() {
        enabled = !enabled;
    }

private:
    bool enabled = false;
    std::shared_ptr<WPI_TalonSRX> intakeMotor = RobotMap::gyroTalon;
};