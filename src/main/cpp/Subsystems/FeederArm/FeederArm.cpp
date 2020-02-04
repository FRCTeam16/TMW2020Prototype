#include "Subsystems/FeederArm/FeederArm.h"

FeederArm::FeederArm()
{
    armMotorFollower->Follow(*armMotor);
    armMotorFollower->SetInverted(true);

    armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    this->ZeroArmPosition();
    armMotor->ConfigForwardSoftLimitThreshold(10000);
    armMotor->ConfigForwardSoftLimitEnable(true);
    armMotor->ConfigReverseSoftLimitThreshold(5000);
    armMotor->ConfigReverseSoftLimitEnable(true);

/*
    armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    armMotor->ConfigForwardLimitSwitchSource(
        RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
        LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
        RobotMap::,
        0);
    armMotor->ConfigReverseLimitSwitchSource(
        RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
        LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
        RobotMap::,
        0);
*/
}

void FeederArm::Init()
{
    this->RetractClimberArms();

    frc::SmartDashboard::PutNumber("IntakeSpeed", 1.0);
    frc::SmartDashboard::PutNumber("FeederSpeed", 0.8);

    frc::SmartDashboard::PutNumber("Arm.P", armPIDConfig.kP);
    frc::SmartDashboard::PutNumber("Arm.I", armPIDConfig.kI);
    frc::SmartDashboard::PutNumber("Arm.D", armPIDConfig.kD);
    frc::SmartDashboard::PutNumber("Arm.F", armPIDConfig.kFF);

    frc::SmartDashboard::PutNumber("Arm.Setpoint", 0.0);

    // frc::SmartDashboard::PutNumber("Arm.V", 0.0);
    // frc::SmartDashboard::PutNumber("Arm.A", 0.0);
}

void FeederArm::Run()
{
    //-----------------------
    // Climber 
    //-----------------------
    if (!climberMessageSent) {
        climberArms->Set(climberExtended);
        climberMessageSent = true;
    }
    

    //-----------------------
    // Intake 
    //-----------------------
    double intakeSpeed = 0.0;
    if (intakeEnabled) {
        intakeSpeed = frc::SmartDashboard::GetNumber("IntakeSpeed", 1.0);
        if (intakeReversed) {
            intakeSpeed = -intakeSpeed;
        }
    }
    intakeMotor->Set(intakeSpeed);

    //-----------------------
    // Feeder Arm
    //-----------------------
    if (!runArmControlled) {
        armMotor->Set(ControlMode::PercentOutput, armSpeed);
    } else {
        // run magic motion
        // see https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B/MotionMagic/src/main/cpp/Robot.cpp
        armPIDConfig.kP = frc::SmartDashboard::GetNumber("Arm.P", 0.0);
        armPIDConfig.kI = frc::SmartDashboard::GetNumber("Arm.I", 0.0);
        armPIDConfig.kD = frc::SmartDashboard::GetNumber("Arm.D", 0.0);
        armPIDConfig.kFF = frc::SmartDashboard::GetNumber("Arm.F", 0.0);
        armSetpoint = frc::SmartDashboard::GetNumber("Arm.Setpoint", 0.0);

        armMotor->Config_kP(0, armPIDConfig.kP, 20);
        armMotor->Config_kI(0, armPIDConfig.kI, 20);
        armMotor->Config_kD(0, armPIDConfig.kD, 20);
        armMotor->Config_kF(0, armPIDConfig.kFF, 20);


        armMotor->Set(ControlMode::Position, armSetpoint);
    }
    frc::SmartDashboard::PutNumber("Arm.Position", armMotor->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Arm.Speed", armSpeed);
}

void FeederArm::Instrument()
{
    
}