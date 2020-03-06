#include "Subsystems/FeederArm/FeederArm.h"
#include "Util/BSPrefs.h"


FeederArm::FeederArm()
{
    armMotorFollower->Follow(*armMotor);
    armMotorFollower->SetInverted(true);

    auto prefs = BSPrefs::GetInstance();

    // Configure arm limits and brake modes
    armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
    armMotor->ConfigForwardSoftLimitThreshold(prefs->GetDouble("FeederArm.Pos.FwdLimit", -5000));;
    armMotor->ConfigReverseSoftLimitThreshold(prefs->GetDouble("FeederArm.Pos.RevLimit",-160000));
    armMotor->ConfigForwardSoftLimitEnable(true);
    armMotor->ConfigReverseSoftLimitEnable(true);

    armMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    armMotorFollower->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    //------------------
    // Configure Arm PID
    //------------------
    armPIDConfig.P = prefs->GetDouble("FeederArm.PID.P", 0.01999998);
    armPIDConfig.I = prefs->GetDouble("FeederArm.PID.I", 0.000001);
    armPIDConfig.D = prefs->GetDouble("FeederArm.PID.D", 0.0);
    armPIDConfig.F = prefs->GetDouble("FeederArm.PID.F", 0.0);
    armPIDConfig.acceleration = prefs->GetDouble("FeederArm.PID.Acceleration", 120000);
    armPIDConfig.velocity = prefs->GetDouble("FeederArm.PID.Velocity", 10000);

    armMotor->Config_kP(0, armPIDConfig.P);
    armMotor->Config_kI(0, armPIDConfig.I);
    armMotor->Config_kD(0, armPIDConfig.D);
    armMotor->Config_kF(0, armPIDConfig.F);
    armMotor->ConfigMotionCruiseVelocity(armPIDConfig.velocity);
    armMotor->ConfigMotionAcceleration(armPIDConfig.acceleration);

    this->ZeroArmPosition();

    frc::SmartDashboard::SetDefaultNumber("FeederArm.IntakeSpeed", 1.0);
    frc::SmartDashboard::SetDefaultNumber("FeederArm.IntakeSpeed.ColorWheel", 0.2);
    frc::SmartDashboard::SetDefaultNumber("Arm.Setpoint", 0.0);

    jamThreshold = BSPrefs::GetInstance()->GetDouble("Intake.Jam.Threshold", 25);
}

void FeederArm::Init()
{
    this->RetractClimberArms();
    armSetpoint = 0.0;
    runArmControlled = false;
    intakeColorWheelMode = false;
    intakeJamDetected = false;

    auto prefs = BSPrefs::GetInstance();
    armPositions[Position::kZero] = 0;
    armPositions[Position::kDown] = prefs->GetDouble("FeederArm.Pos.Down",-10000);
    armPositions[Position::kShortShot] = prefs->GetDouble("FeederArm.Pos.ShortShot", -65000);
    armPositions[Position::kPlayerStation] = prefs->GetDouble("FeederArm.Pos.PlayerStation", -75000);
    armPositions[Position::kVertical] = prefs->GetDouble("FeederArm.Pos.Vertical", -105000);
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
        climberLeftArm->Set(climberExtended);
        climberRightArm->Set(climberExtended);
        climberMessageSent = true;
    }

    //-----------------------
    // Intake
    //-----------------------
    
    double intakeSpeed = 0.0;
    if (intakeEnabled || intakeColorWheelMode)
    {
        intakeSpeed = intakeColorWheelMode ? 
            colorModeSpeed :
            frc::SmartDashboard::GetNumber("FeederArm.IntakeSpeed", 1.0);
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
        armMotor->Set(ControlMode::MotionMagic, armSetpoint);
    }
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

void FeederArm::SetArmPosition(Position position)
{
    auto iter = armPositions.find(position);
    if (iter == armPositions.end())
    {
        std::cout << "*** ERROR: FeederArm::SetArmPosition unable to "
                  << "locate requested position: " << position << "\n";
    }
    else
    {
        armSetpoint = iter->second;
        std::cout << "FeederArm::SetArmPosition: " << armSetpoint << "\n";
        runArmControlled = true;
        frc::SmartDashboard::PutNumber("Arm.Setpoint", armSetpoint);
    }
}

void FeederArm::ZeroArmPosition(int position)
{
    std::cout << "FeederArm::ZeroArmPosition(" << position << ")\n";
    armMotor->SetSelectedSensorPosition(position, 0, 50);
}

bool FeederArm::IsArmInPosition()
{
    const double allowedError = BSPrefs::GetInstance()->GetDouble("FeederArm.Pos.AllowedError", 3000);
    const double currentPosition = armMotor->GetSelectedSensorPosition();
    return fabs((currentPosition - armSetpoint) < allowedError);
}

double FeederArm::GetCurrentSetPoint()
{
    return armSetpoint;
}


void FeederArm::SetArmBrakeMode(bool brakeEnabled)
{
    ctre::phoenix::motorcontrol::NeutralMode neutralMode;
    if (brakeEnabled)
    {
        std::cout << "FeederArm::ToggleArmBreakMode() -> setting to coast\n";
        neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
    }
    else
    {
        std::cout << "FeederArm::ToggleArmBreakMode() -> setting to brake\n";
        neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
    }
    armMotor->SetNeutralMode(neutralMode);
    armMotorFollower->SetNeutralMode(neutralMode);
}

/*****************************************************************************/

void FeederArm::StartIntake(bool reverse)
{
    intakeEnabled = true;
    intakeReversed = reverse;
    intakeColorWheelMode = false;
}

void FeederArm::StartIntakeForColorSpin(double speed)
{
    colorModeSpeed = speed;
    intakeEnabled = true;
    intakeColorWheelMode = true;
}

void FeederArm::StopIntakeForColorSpin()
{
    intakeEnabled = false;
    intakeColorWheelMode = false;
}


void FeederArm::StopIntake()
{
    intakeEnabled = false;
}

bool FeederArm::IsIntakeJamDetected()
{
    return intakeJamDetected;
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

    const double currentIntakeAmps = powerDistributionPanel->GetCurrent(10);
    frc::SmartDashboard::PutNumber("Intake.Amps.Out", currentIntakeAmps);

    // FIXME: Move to run loop
    const double filterOutput = intakeSpikeDetector.Calculate(currentIntakeAmps);
    frc::SmartDashboard::PutNumber("Intake.Filter.Out", filterOutput);
    intakeJamDetected = (filterOutput > jamThreshold);
    frc::SmartDashboard::PutBoolean("Intake.Jam.Detected", intakeJamDetected);
}