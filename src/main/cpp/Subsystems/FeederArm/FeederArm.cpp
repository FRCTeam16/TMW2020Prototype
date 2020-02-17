#include "Subsystems/FeederArm/FeederArm.h"

static int kVelocity = 10000;
static int kAcceleration = 120000;

FeederArm::FeederArm()
{
    armMotorFollower->Follow(*armMotor);
    armMotorFollower->SetInverted(true);

    this->ZeroArmPosition();

    armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
    armMotor->ConfigForwardSoftLimitThreshold(-5000);
    armMotor->ConfigReverseSoftLimitThreshold(-130000);
    armMotor->ConfigForwardSoftLimitEnable(true);
    armMotor->ConfigReverseSoftLimitEnable(true);

    // FIXME: Is this still eneded
    armMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    armMotorFollower->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    armPIDConfig.kP = 0.01999998;
    armPIDConfig.kI = 0.000001;

    frc::SmartDashboard::SetDefaultNumber("IntakeSpeed", 1.0);

    frc::SmartDashboard::SetDefaultNumber("Arm.P", armPIDConfig.kP);
    frc::SmartDashboard::SetDefaultNumber("Arm.I", armPIDConfig.kI);
    frc::SmartDashboard::SetDefaultNumber("Arm.D", armPIDConfig.kD);
    frc::SmartDashboard::SetDefaultNumber("Arm.F", armPIDConfig.kFF);

    frc::SmartDashboard::SetDefaultNumber("Arm.Setpoint", 0.0);

    frc::SmartDashboard::SetDefaultNumber("Arm.V", kVelocity);
    frc::SmartDashboard::SetDefaultNumber("Arm.A", kAcceleration);
}

void FeederArm::Init()
{
    this->RetractClimberArms();
    armSetpoint = 0.0;
    runArmControlled = false; // FIXME: Determine if we want to start controlled or not
}

void FeederArm::InitTeleop()
{
    armMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    armMotorFollower->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void FeederArm::InitAuto()
{
    armMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    armMotorFollower->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void FeederArm::Run()
{
    //-----------------------
    // Climber
    //-----------------------
    if (!climberMessageSent)
    {
        climberArms->Set(climberExtended);
        climberMessageSent = true;
    }

    //-----------------------
    // Intake
    //-----------------------
    double intakeSpeed = 0.0;
    if (intakeEnabled)
    {
        intakeSpeed = frc::SmartDashboard::GetNumber("IntakeSpeed", 1.0);
        if (intakeReversed)
        {
            intakeSpeed = -intakeSpeed;
        }
    }
    intakeMotor->Set(intakeSpeed);

    //-----------------------
    // Feeder Arm
    //-----------------------
    if (!runArmControlled)
    {
        armMotor->Set(ControlMode::PercentOutput, armSpeed);
    }
    else
    {
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

/*****************************************************************************/

void FeederArm::RunArm(double speed)
{
    runArmControlled = false;
    armSpeed = speed;
}

void FeederArm::RunArmControlled()
{
    runArmControlled = true;
}

void FeederArm::DebugSetPoint(double _setpoint)
{
    std::cout << "**** FeederARm::DebugSetPoint: " << _setpoint << "\n";
    runArmControlled = true;
    armSetpoint = _setpoint;
    frc::SmartDashboard::PutNumber("Arm.Setpoint", armSetpoint);
}

void FeederArm::ZeroArmPosition()
{
    armMotor->SetSelectedSensorPosition(0, 0, 50);
}

/*****************************************************************************/

void FeederArm::StartIntake(bool reverse)
{
    intakeEnabled = true;
    intakeReversed = reverse;
}

void FeederArm::StopIntake()
{
    intakeEnabled = false;
}

/*****************************************************************************/

void FeederArm::ExtendClimberArms()
{
    climberMessageSent = false;
    climberExtended = true;
}

void FeederArm::RetractClimberArms()
{
    climberMessageSent = false;
    climberExtended = false;
}

/*****************************************************************************/

/*****************************************************************************/

void FeederArm::Instrument()
{
    frc::SmartDashboard::PutNumber("Arm.Amps.Out", armMotor->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Arm.Position", armMotor->GetSelectedSensorPosition());
}