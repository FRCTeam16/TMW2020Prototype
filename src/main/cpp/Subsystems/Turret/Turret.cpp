#include "Subsystems/Turret/Turret.h"

const double kMinRPMToShoot = 2500.0;

Turret::Turret(std::shared_ptr<VisionSystem> visionSystem)
    : visionSystem(visionSystem), turretRotation(TurretRotation{visionSystem})
{
    shooterMotor->SetInverted(false);
    shooterMotor->SetClosedLoopRampRate(0.5);
    shooterMotorFollower->Follow(*shooterMotor, true);

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

    frc::SmartDashboard::PutNumber("SetPoint1", 0);

    //---------------------------------
    // Feeder PID Controls and Settings
    //---------------------------------
    frc::SmartDashboard::PutNumber("Feeder.Preload.Time", 0.4);
    frc::SmartDashboard::PutNumber("Feeder.Preload.Speed", -0.2);

    frc::SmartDashboard::SetDefaultNumber("Feeder.RPM", -4200.0);
    frc::SmartDashboard::PutNumber("Feeder.PID.P", 0.00001);
    frc::SmartDashboard::PutNumber("Feeder.PID.I", 0.0);
    frc::SmartDashboard::PutNumber("Feeder.PID.D", 0.0);
    frc::SmartDashboard::PutNumber("Feeder.PID.F", 0.000185);


    frc::SmartDashboard::PutNumber("Turret.RPM.Long", 4250);
    frc::SmartDashboard::PutNumber("Turret.RPM.Short", 3800);

    //debug
    frc::SmartDashboard::PutNumber("Shooter.Get.Out", shooterMotor->Get());


    std::cout << "Turret initialized\n";
    
}

void Turret::Init()
{
    shooterEnabled = false;
    this->SetLidToLongShot();
    turretRotation.Init();
}

void Turret::Run()
{
    const double now = frc::Timer::GetFPGATimestamp();

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
    turretRotation.Run();

    //----------------
    // Feeder Control
    //----------------
    if (preloadFeederRunning)
    {
        double feederSpeed = 0.0;
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
            feederSpeed = 0.0;
        }
        std::cout << "feeder preload block\n";
        feederMotor->Set(feederSpeed);
    }
    /*else if (feederAndShooterReversed) {
        double reverseSpeed = -1.0 * copysign(1.0, frc::SmartDashboard::GetNumber("Feeder.RPM", -4200));
        feederMotor->Set(reverseSpeed);
    }*/
    else if (feederEnabled)
    {
        UpdateFeederPID();
        if (shooterMotor->GetEncoder().GetVelocity() > kMinRPMToShoot)
        {
            double feederRPM = frc::SmartDashboard::GetNumber("Feeder.RPM", -4200);
            if (feederReversed)
            {
                feederRPM = -feederRPM;
            }
            feederMotor->GetPIDController().SetReference(feederRPM, rev::ControlType::kVelocity);

            // double feederSpeed = frc::SmartDashboard::GetNumber("FeederSpeed", -.70);
            // feederMotor->Set(0.0);
        }
        else
        {
            std::cout << "!!! SHOOTER NOT ENABLED, IGNORING REQUEST TO FIRE FEEDER !!!\n";
            feederMotor->Set(0.0);
        }
    } else {
        // default
        feederMotor->Set(0.0);
    }


    //----------------
    // Shooter Control
    //----------------
    UpdateShooterPID();
    /*if (feederAndShooterReversed) {
        double reverseSpeed = -1.0 * copysign(1.0, frc::SmartDashboard::GetNumber("Turret.RPM.Long", 5000));
        shooterMotor->Set(reverseSpeed);
    }
    else*/ if (shooterEnabled)
    {
        rev::CANPIDController shooterPIDController = shooterMotor->GetPIDController();
        double shooterRPM = shooterPIDConfig.kRpm1;
        frc::SmartDashboard::PutNumber("Velocity1", shooterMotor->GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("ShooterSetpoint", shooterRPM);
        frc::SmartDashboard::PutNumber("Shooter Amps", shooterMotor->GetOutputCurrent());
        frc::SmartDashboard::PutNumber("Shooter Output", shooterMotor->GetAppliedOutput());
        frc::SmartDashboard::PutNumber("Shooter.Get.Out", shooterMotor->Get());
        shooterPIDController.SetReference(shooterRPM, rev::ControlType::kVelocity);
    }
    else
    {
        // Go open loop
        shooterMotor->Set(0.0);
    }
}

TurretRotation& Turret::GetTurretRotation() {
    return turretRotation;
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

void Turret::SetFeederAndShooterReversed(bool reversed)
{
    feederAndShooterReversed = reversed;
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
    frc::SmartDashboard::PutBoolean("ShooterEnabled", shooterEnabled);
    frc::SmartDashboard::PutNumber("Feeder Amps", feederMotor->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Feeder Out RPM", feederMotor->GetEncoder().GetVelocity());
    turretRotation.Instrument();
}

// ***************************************************************************/


void Turret::UpdateFeederPID()
{
    double p = frc::SmartDashboard::GetNumber("Feeder.PID.P", 0.0);
    double i = frc::SmartDashboard::GetNumber("Feeder.PID.I", 0.0);
    double d = frc::SmartDashboard::GetNumber("Feeder.PID.D", 0.0);
    double f = frc::SmartDashboard::GetNumber("Feeder.PID.F", 0.0);

    auto feederPID = feederMotor->GetPIDController();
    feederPID.SetP(p);
    feederPID.SetI(i);
    feederPID.SetD(d);
    feederPID.SetFF(f);
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
