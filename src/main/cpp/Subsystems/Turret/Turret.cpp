#include "Subsystems/Turret/Turret.h"

const double kMinRPMToShoot = 2500.0;

Turret::Turret(std::shared_ptr<VisionSystem> visionSystem)
    : visionSystem(visionSystem)
{
    shooterMotor->SetInverted(false);
    shooterMotor->SetClosedLoopRampRate(0.5);
    shooterMotorFollower->Follow(*shooterMotor, true);
    std::cout << "Turret shooterMotor: " << shooterMotor.get() << "\n";
    std::cout << "Turret shooterMotorFollower: " << shooterMotorFollower.get() << "\n";
    std::cout << "Turret initialized\n";

    //-------------------------------
    // Shooter PID Dashboard Controls
    //-------------------------------
    shooterPIDConfig.kP = 0.0004;
    shooterPIDConfig.kFF = 0.000173;

    frc::SmartDashboard::PutNumber("#1 P Gain", shooterPIDConfig.kP);
    frc::SmartDashboard::PutNumber("#1 I Gain", shooterPIDConfig.kI);
    frc::SmartDashboard::PutNumber("#1 D Gain", shooterPIDConfig.kD);
    frc::SmartDashboard::PutNumber("#1 I Zone", shooterPIDConfig.kIz);
    frc::SmartDashboard::PutNumber("#1 Feed Forward", shooterPIDConfig.kFF);
    frc::SmartDashboard::PutNumber("#1 Max Output", shooterPIDConfig.kMaxOutput);
    frc::SmartDashboard::PutNumber("#1 Min Output", shooterPIDConfig.kMinOutput);

    frc::SmartDashboard::PutNumber("Turret.Setpoint", turretMotor->GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Turret.PID.P", 2.0);
    frc::SmartDashboard::PutNumber("Turret.PID.I", 0);
    frc::SmartDashboard::PutNumber("Turret.PID.D", 0);
    frc::SmartDashboard::PutNumber("Turret.RPM.Long", 5000);
    frc::SmartDashboard::PutNumber("Turret.RPM.Short", 3800);

    frc::SmartDashboard::PutNumber("SetPoint1", 0);

    // Feeder Control
    frc::SmartDashboard::SetDefaultNumber("FeederSpeed", -1.0);
    frc::SmartDashboard::PutNumber("Feeder.Speed.Out", 0.0);
    frc::SmartDashboard::PutNumber("Feeder.Preload.Time", 0.4);
    frc::SmartDashboard::PutNumber("Feeder.Preload.Speed", -0.2);
}

void Turret::Init()
{
    shooterEnabled = false;
    visionTrackingEnabled = false;
    openLoopMessage = false;
    this->SetLidToLongShot();
    turretSetpoint = turretMotor->GetEncoder().GetPosition();
}

void Turret::Run()
{
    const double now = frc::Timer::GetFPGATimestamp();
    auto visionInfo = visionSystem->GetLastVisionInfo();

    //-------------------
    // Turret Lid Control
    //-------------------
    if (!lidTopMessageSent)
    {
        lidTop->Set(lidTopShort);
        lidTopMessageSent = true;
    }

    //----------------
    // Turret Control
    //----------------
    // std::cout << "Turret(tpc=" << turretPositionControl << ", ol=" << openLoopMessage << ", vt=" << visionTrackingEnabled << ")\n";
    if (!visionTrackingEnabled || openLoopMessage || turretPositionControl)
    {
        if (openLoopMessage)
        {
            turretMotor->Set(turretSpeed);
            openLoopMessage = false;
        }
        else if (turretPositionControl)
        {
            UpdateTurretPID();
            turretMotor->GetPIDController().SetReference(turretSetpoint, rev::ControlType::kPosition);
        }
    }
    else
    {
        // Vision Loop control
        double speed = 0.0;

        if (visionInfo->hasTarget)
        {
            if (visionTargetAcquiredTime < 0)
            {
                visionTargetAcquiredTime = frc::Timer::GetFPGATimestamp();
            }
            speed = -visionInfo->xSpeed;
        }
        else
        {
            visionTargetAcquiredTime = -1.0;
        }
        turretMotor->Set(speed);
    }

    //----------------
    // Feeder Control
    //----------------
    double feederSpeed = 0.0;
    if (preloadFeederRunning)
    {
        const double MAX_ELAPSED = frc::SmartDashboard::GetNumber("Feeder.Preload.Time", 0.4);
        const double PRELOAD_SPEED = frc::SmartDashboard::GetNumber("Feeder.Preload.Speed", -0.2);
        if ((now - preloadFeederStarted) < MAX_ELAPSED)
        {
            std::cout << "~~ Preload Running ~~\n";
            feederSpeed = PRELOAD_SPEED;
        }
        else
        {
            std::cout << "~~ Preload Stopped ~~\n";
            preloadFeederRunning = false;
        }
    }
    else if (feederEnabled)
    {
        if (shooterMotor->GetEncoder().GetVelocity() > kMinRPMToShoot)
        {
            feederSpeed = frc::SmartDashboard::GetNumber("FeederSpeed", -.70);
            if (feederReversed)
            {
                feederSpeed = -feederSpeed;
            }
        }
        else
        {
            std::cout << "!!! SHOOTER NOT ENABLED, IGNORING REQUEST TO FIRE FEEDER !!!\n";
        }
    }
    feederMotor->Set(feederSpeed);
    frc::SmartDashboard::PutNumber("Feeder.Speed.Out", feederSpeed);

    //----------------
    // Shooter Control
    //----------------
    UpdateShooterPID();
    if (shooterEnabled)
    {
        rev::CANPIDController shooterPIDController = shooterMotor->GetPIDController();
        double shooterRPM = shooterPIDConfig.kRpm1;
        frc::SmartDashboard::PutNumber("Velocity1", shooterMotor->GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("ShooterSetpoint", shooterRPM);
        frc::SmartDashboard::PutNumber("Shooter Amps", shooterMotor->GetOutputCurrent());
        frc::SmartDashboard::PutNumber("Shooter Output", shooterMotor->GetAppliedOutput());
        shooterPIDController.SetReference(shooterRPM, rev::ControlType::kVelocity);
    }
    else
    {
        // Go open loop
        shooterMotor->Set(0.0);
    }
}

// ***************************************************************************/

void Turret::SetOpenLoopTurretSpeed(double _speed)
{
    openLoopMessage = true;
    turretSpeed = _speed;
}

void Turret::OpenLoopHaltTurret()
{
    if (turretSpeed != 0.0)
    {
        openLoopMessage = true;
    }
    turretSpeed = 0.0;
}

void Turret::SetTurretSetpoint(double setpoint)
{
    std::cout << "*************** TURRET::SetTurretSetpoint(" << setpoint << ")\n";
    turretSetpoint = setpoint;
    frc::SmartDashboard::PutNumber("Turret.Setpoint", turretSetpoint);
}

bool Turret::IsTurretInPosition()
{
    double error = fabs(turretSetpoint - turretMotor->GetEncoder().GetPosition());
    return (error < 1);
}

void Turret::EnableTurretPositionControl(bool control)
{
    turretPositionControl = control;
}

// ***************************************************************************/

void Turret::EnableVisionTracking()
{
    visionTrackingEnabled = true;
    turretPositionControl = false;
}

void Turret::DisableVisionTracking()
{
    visionTrackingEnabled = false;
}

void Turret::ToggleVisionTracking()
{
    if (visionTrackingEnabled)
    {
        // turn off
        visionSystem->DisableVisionTracking();
        this->DisableVisionTracking();
    }
    else
    {
        // turn on
        visionSystem->EnableVisionTracking();
        this->EnableVisionTracking();
    }
}

bool Turret::IsVisionTracking()
{
    return visionTrackingEnabled;
}

// ***************************************************************************/

void Turret::SetShooterEnabled(bool _enabled)
{
    shooterEnabled = _enabled;
}

void Turret::ToggleShooterEnabled()
{
    shooterEnabled = !shooterEnabled;
}

// ***************************************************************************/

void Turret::StartFeeder(bool reverse)
{
    feederEnabled = true;
    feederReversed = reverse;
}

void Turret::StopFeeder()
{
    feederEnabled = false;
}

void Turret::PreloadBall()
{
    if (!preloadFeederRunning)
    {
        preloadFeederRunning = true;
        preloadFeederStarted = frc::Timer::GetFPGATimestamp();
    }
}

// ***************************************************************************/

void Turret::SetLidToLongShot()
{
    lidTopShort = false;
    lidTopMessageSent = false;
}

void Turret::SetLidToShortShot()
{
    lidTopShort = true;
    lidTopMessageSent = false;
}

// ***************************************************************************/

void Turret::Instrument()
{
    frc::SmartDashboard::PutNumber("Turret Position", turretMotor->GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Turret Velocity", turretMotor->GetEncoder().GetVelocity());
    frc::SmartDashboard::PutBoolean("ShooterEnabled", shooterEnabled);
    frc::SmartDashboard::PutNumber("Feeder Amps", feederMotor->GetOutputCurrent());
}

// ***************************************************************************/

void Turret::UpdateTurretPID()
{
    double setpoint = frc::SmartDashboard::GetNumber("Turret.Setpoint", turretSetpoint);
    double p = frc::SmartDashboard::GetNumber("Turret.PID.P", 2.0);
    double i = frc::SmartDashboard::GetNumber("Turret.PID.I", 0);
    double d = frc::SmartDashboard::GetNumber("Turret.PID.D", 0);

    if (setpoint != turretSetpoint)
    {
        this->SetTurretSetpoint(setpoint);
    }

    auto turretPID = turretMotor->GetPIDController();
    turretPID.SetP(p);
    turretPID.SetI(i);
    turretPID.SetD(d);
}

void Turret::UpdateShooterPID()
{
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("#1 P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("#1 I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("#1 D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("#1 I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("#1 Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("#1 Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("#1 Min Output", 0);

    double rpm = 0.0;
    if (lidTopShort)
    {
        rpm = frc::SmartDashboard::GetNumber("Turret.RPM.Short", 3800);
    }
    else
    {
        rpm = frc::SmartDashboard::GetNumber("Turret.RPM.Long", 5000);
    }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    auto shooterPIDController = shooterMotor->GetPIDController();
    if ((p != shooterPIDConfig.kP))
    {
        shooterPIDController.SetP(p);
        shooterPIDConfig.kP = p;
    }
    if ((i != shooterPIDConfig.kI))
    {
        shooterPIDController.SetI(i);
        shooterPIDConfig.kI = i;
    }
    if ((d != shooterPIDConfig.kD))
    {
        shooterPIDController.SetD(d);
        shooterPIDConfig.kD = d;
    }
    if ((iz != shooterPIDConfig.kIz))
    {
        shooterPIDController.SetIZone(iz);
        shooterPIDConfig.kIz = iz;
    }
    if ((ff != shooterPIDConfig.kFF))
    {
        shooterPIDController.SetFF(ff);
        shooterPIDConfig.kFF = ff;
    }
    if ((max != shooterPIDConfig.kMaxOutput) || (min != shooterPIDConfig.kMinOutput))
    {
        shooterPIDController.SetOutputRange(min, max);
        shooterPIDConfig.kMinOutput = min;
        shooterPIDConfig.kMaxOutput = max;
    }
    if ((rpm != shooterPIDConfig.kRpm1))
    {
        shooterPIDConfig.kRpm1 = rpm;
    }
}
