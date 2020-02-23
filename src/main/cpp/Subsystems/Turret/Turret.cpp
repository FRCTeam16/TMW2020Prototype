#include "Subsystems/Turret/Turret.h"

const double kMinRPMToShoot = 2500.0;

Turret::Turret(std::shared_ptr<VisionSystem> visionSystem)
    : visionSystem(visionSystem), turretRotation(TurretRotation{visionSystem})
{
    shooterMotor->SetInverted(false);
    shooterMotor->SetClosedLoopRampRate(0.5);
    shooterMotorFollower->Follow(*shooterMotor, true);

    auto prefs = BSPrefs::GetInstance();

    //-------------------------------
    // Shooter PID Dashboard Controls
    //-------------------------------
    shooterConfig.P = prefs->GetDouble("Shooter.PID.P", shooterConfig.P);
    shooterConfig.I = prefs->GetDouble("Shooter.PID.I", shooterConfig.I);
    shooterConfig.D = prefs->GetDouble("Shooter.PID.D", shooterConfig.D);
    shooterConfig.F = prefs->GetDouble("Shooter.PID.F", shooterConfig.F);
    frc::SmartDashboard::SetDefaultNumber("Shooter.RPM", 4500);


    //---------------------------------
    // Feeder PID Controls and Settings
    //---------------------------------
    feederConfig.preloadTime = prefs->GetDouble("Feeder.Preload.Time", feederConfig.preloadTime);
    feederConfig.preloadSpeed = prefs->GetDouble("Feeder.Preload.Speed", feederConfig.preloadTime);
    feederConfig.P = prefs->GetDouble("Feeder.PID.P", feederConfig.P);
    feederConfig.I = prefs->GetDouble("Feeder.PID.I", feederConfig.I);
    feederConfig.D = prefs->GetDouble("Feeder.PID.D", feederConfig.D);
    feederConfig.F = prefs->GetDouble("Feeder.PID.F", feederConfig.F);
    frc::SmartDashboard::SetDefaultNumber("Feeder.RPM", -4200.0);

    InitShootingProfiles();
    SetShootingProfile(ShootingProfile::kLong);
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
        if ((now - preloadFeederStarted) < feederConfig.preloadTime)
        {
            std::cout << "~~ Preload Running ~~\n";
            feederSpeed = feederConfig.preloadSpeed;
        }
        else
        {
            std::cout << "~~ Preload Stopped ~~\n";
            preloadFeederRunning = false;
            feederSpeed = 0.0;
        }
        feederMotor->Set(feederSpeed);
    }
    /*else if (feederAndShooterReversed) {
        double reverseSpeed = -1.0 * copysign(1.0, frc::SmartDashboard::GetNumber("Feeder.RPM", -4200));
        feederMotor->Set(reverseSpeed);
    }*/
    else if (feederEnabled)
    {
        // UpdateFeederPID();
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
    // UpdateShooterPID();
    /*if (feederAndShooterReversed) {
        double reverseSpeed = -1.0 * copysign(1.0, frc::SmartDashboard::GetNumber("Turret.RPM.Long", 5000));
        shooterMotor->Set(reverseSpeed);
    }
    else*/ if (shooterEnabled)
    {
        rev::CANPIDController shooterPIDController = shooterMotor->GetPIDController();
        double shooterRPM = frc::SmartDashboard::GetNumber("Shooter.RPM", 4500.0);
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


void Turret::SetShootingProfile(ShootingProfile profile)
{
    if (shootingProfiles.find(profile) != shootingProfiles.end()) {
        auto config = shootingProfiles[profile];
        frc::SmartDashboard::PutNumber("Shooter.RPM", config.shooterRPM);
        frc::SmartDashboard::PutNumber("Feeder.RPM", config.feederRPM);
        
    } else {
        std::cout << "*** Turret::SetShootingProfile: Unknown profile requested -> " << profile << "\n";
    }
}


void Turret::InitShootingProfiles()
{
    auto prefs = BSPrefs::GetInstance();
    ShootingProfileConfig shortCfg, mediumCfg, longCfg;

    shortCfg.shooterRPM = prefs->GetDouble("ShootingProfile.Short.Shooter", 4250);
    shortCfg.feederRPM = prefs->GetDouble("ShootingProfile.Short.Feeder", 5000);
    shootingProfiles[ShootingProfile::kShort] = shortCfg;

    mediumCfg.shooterRPM = prefs->GetDouble("ShootingProfile.Medium.Shooter", 4250);
    mediumCfg.feederRPM = prefs->GetDouble("ShootingProfile.Medium.Feeder", 5000);
    shootingProfiles[ShootingProfile::kMedium] = mediumCfg;

    longCfg.shooterRPM = prefs->GetDouble("ShootingProfile.Long.Shooter", 4500);
    longCfg.feederRPM = prefs->GetDouble("ShootingProfile.Long.Feeder", 4000);
    shootingProfiles[ShootingProfile::kMedium] = longCfg;

}

// ***************************************************************************/

void Turret::Instrument()
{
    frc::SmartDashboard::PutBoolean("ShooterEnabled", shooterEnabled);
    frc::SmartDashboard::PutNumber("Shooter Out RPM", shooterMotor->GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Amps", shooterMotor->GetOutputCurrent());
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
    double p = frc::SmartDashboard::GetNumber("Shooter.PID.P", 0.0);
    double i = frc::SmartDashboard::GetNumber("Shooter.PID.I", 0.0);
    double d = frc::SmartDashboard::GetNumber("Shooter.PID.D", 0.0);
    double f = frc::SmartDashboard::GetNumber("Shooter.PID.F", 0.0);

    auto shooterPID = shooterMotor->GetPIDController();
    shooterPID.SetP(p);
    shooterPID.SetI(i);
    shooterPID.SetD(d);
    shooterPID.SetFF(f);
}
