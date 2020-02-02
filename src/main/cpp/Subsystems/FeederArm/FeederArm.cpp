#include "Subsystems/FeederArm/FeederArm.h"

FeederArm::FeederArm()
{
    armMotorFollower->Follow(*armMotor);
    armMotorFollower->SetInverted(true);
}

void FeederArm::Init()
{
    frc::SmartDashboard::PutNumber("IntakeSpeed", 1.0);
    frc::SmartDashboard::PutNumber("FeederSpeed", -0.8);
    this->RetractClimberArms();
}

void FeederArm::Run()
{
    double intakeSpeed = 0.0;
    if (intakeEnabled) {
        intakeSpeed = frc::SmartDashboard::GetNumber("IntakeSpeed", 1.0);
        if (intakeReversed) {
            intakeSpeed = -intakeSpeed;
        }
    }
    intakeMotor->Set(intakeSpeed);

    if (!runArmControlled) {
        armMotor->Set(ControlMode::PercentOutput, armSpeed);
    } else {
        // run magic motion
        // see https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B/MotionMagic/src/main/cpp/Robot.cpp
    }
    frc::SmartDashboard::PutNumber("ArmPos", armMotor->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("ArmPercent", armSpeed);
}

void FeederArm::Instrument()
{
    
}