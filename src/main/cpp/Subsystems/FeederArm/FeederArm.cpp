#include "Subsystems/FeederArm/FeederArm.h"

static int kVelocity = 10000;
static int kAcceleration = 120000;

FeederArm::FeederArm()
{
    armMotorFollower->Follow(*armMotor);
    armMotorFollower->SetInverted(true);

    this->ZeroArmPosition();

    armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
    armMotor->ConfigForwardSoftLimitThreshold(130000);
    armMotor->ConfigForwardSoftLimitEnable(true);
    armMotor->ConfigReverseSoftLimitThreshold(5000);
    armMotor->ConfigReverseSoftLimitEnable(true);
    // armMotor->ConfigPeakOutputForward(0.5);
    // armMotor->ConfigPeakOutputReverse(-0.5);
    

    armPIDConfig.kP = 0.01999998;

}

void FeederArm::Init()
{
    this->RetractClimberArms();
    armSetpoint = 0.0;
    runArmControlled = false;   // FIXME: Determine if this is true

    frc::SmartDashboard::PutNumber("IntakeSpeed", 1.0);
    frc::SmartDashboard::PutNumber("FeederSpeed", -1.0);

    frc::SmartDashboard::PutNumber("Arm.P", armPIDConfig.kP);
    frc::SmartDashboard::PutNumber("Arm.I", armPIDConfig.kI);
    frc::SmartDashboard::PutNumber("Arm.D", armPIDConfig.kD);
    frc::SmartDashboard::PutNumber("Arm.F", armPIDConfig.kFF);

    frc::SmartDashboard::PutNumber("Arm.Setpoint", 0.0);

    frc::SmartDashboard::PutNumber("Arm.V", kVelocity);
    frc::SmartDashboard::PutNumber("Arm.A", kAcceleration);
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

        double velocity = frc::SmartDashboard::GetNumber("Arm.V", kVelocity);
        double acceleration = frc::SmartDashboard::GetNumber("Arm.A", kAcceleration);

        armMotor->Config_kP(0, armPIDConfig.kP, 2);
        armMotor->Config_kI(0, armPIDConfig.kI, 2);
        armMotor->Config_kD(0, armPIDConfig.kD, 2);
        armMotor->Config_kF(0, armPIDConfig.kFF, 2);
        armMotor->ConfigMotionCruiseVelocity(velocity, 2);
        armMotor->ConfigMotionAcceleration(acceleration, 2);

        armMotor->Set(ControlMode::MotionMagic, armSetpoint);
    }
    frc::SmartDashboard::PutNumber("Arm.Position", armMotor->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Arm.Speed", armSpeed);
}

void FeederArm::Instrument()
{
    
}