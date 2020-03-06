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
    shooterConfig.P = prefs->GetDouble("Shooter.PID.P", 0.0004);
    shooterConfig.I = prefs->GetDouble("Shooter.PID.I", 0.0);
    shooterConfig.D = prefs->GetDouble("Shooter.PID.D", 0.0);
    shooterConfig.F = prefs->GetDouble("Shooter.PID.F", 0.000173);
    frc::SmartDashboard::SetDefaultNumber("Shooter.RPM", 4500);

    auto shooterPID = shooterMotor->GetPIDController();
    shooterPID.SetP(shooterConfig.P);
    shooterPID.SetI(shooterConfig.I);
    shooterPID.SetD(shooterConfig.D);
    shooterPID.SetFF(shooterConfig.F);

    //---------------------------------
    // Feeder PID Controls and Settings
    //---------------------------------
    feederConfig.preloadTime = prefs->GetDouble("Feeder.Preload.Time", 0.4);
    feederConfig.preloadSpeed = prefs->GetDouble("Feeder.Preload.Speed", -0.2);
    feederConfig.P = prefs->GetDouble("Feeder.PID.P", 0.00001);
    feederConfig.I = prefs->GetDouble("Feeder.PID.I", 0.0);
    feederConfig.D = prefs->GetDouble("Feeder.PID.D", 0.0);
    feederConfig.F = prefs->GetDouble("Feeder.PID.F", 0.000185);
    frc::SmartDashboard::SetDefaultNumber("Feeder.RPM", -4200.0);

    auto feederPID = feederMotor->GetPIDController();
    feederPID.SetP(feederConfig.P);
    feederPID.SetI(feederConfig.I);
    feederPID.SetD(feederConfig.D);
    feederPID.SetFF(feederConfig.F);

    //---------------------------------
    // Initial states
    //---------------------------------

    InitShootingProfiles();
    SetShootingProfile(ShootingProfile::kMedium);
    std::cout << "Turret initialized\n"; 
}

void Turret::Init()
{
    shooterEnabled = false;
    feederAndShooterReversed = false;
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
    // UpdateFeederPID();
    const double currentFeederRPM = frc::SmartDashboard::GetNumber("Feeder.RPM", -4200);
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
    else if (feederAndShooterReversed) {
        feederMotor->Set(-currentFeederRPM);
    }
    else if (feederEnabled)
    {
        if (shooterMotor->GetEncoder().GetVelocity() > kMinRPMToShoot)
        {
            double feederRPM = currentFeederRPM;
            if (feederReversed)
            {
                feederRPM = -feederRPM;
            }
            feederMotor->GetPIDController().SetReference(feederRPM, rev::ControlType::kVelocity);

            // double feederSpeed = frc::SmartDashboard::GetNumber("Turret.FeederSpeed", -.70);
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
    // Note that shooter control can override TurretRotation vision tracking state
    // Shooter will talk directly to vision system to enable/disable light when 
    // Turret Rotation has been configured to use vision tracking
    //----------------
    // UpdateShooterPID();
    const double currentShooterRPM = frc::SmartDashboard::GetNumber("Shooter.RPM", 4500.0);
    if (feederAndShooterReversed) {
        shooterMotor->Set(-currentShooterRPM);
    }
    else if (shooterEnabled)
    {
        if (turretRotation.IsVisionTracking()) {
            visionSystem->EnableVisionTracking();
        }
        
        rev::CANPIDController shooterPIDController = shooterMotor->GetPIDController();
        shooterPIDController.SetReference(currentShooterRPM, rev::ControlType::kVelocity);
    }
    else
    {
        if (turretRotation.IsVisionTracking())
        {
            visionSystem->DisableVisionTracking();
        }
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

bool Turret::IsShooterEnabled()
{
    return shooterEnabled;
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
        currentShootingProfile = profile;
        auto config = shootingProfiles[profile];
        frc::SmartDashboard::PutNumber("Shooter.RPM", config.shooterRPM);
        frc::SmartDashboard::PutNumber("Feeder.RPM", config.feederRPM);        
    } else {
        std::cout << "*** Turret::SetShootingProfile: Unknown profile requested -> " << profile << "\n";
    }

    // Update display
    wpi::StringRef profileName = "Unknown";
    switch (profile)
    {
    case ShootingProfile::kLong :
        profileName = "Long";
        break;
    case ShootingProfile::kMedium:
        profileName = "Medium";
        break;
    case ShootingProfile::kShort:
        profileName = "Short";
        break;
    default:
        break;
    }
    frc::SmartDashboard::PutString("Shooting Profile", profileName);
}

ShootingProfile Turret::GetCurrentShootingProfile()
{
    return currentShootingProfile;
}


void Turret::InitShootingProfiles()
{
    auto prefs = BSPrefs::GetInstance();
    ShootingProfileConfig shortCfg, mediumCfg, longCfg, autoFadeCfg;
    shootingProfiles.clear();

    shortCfg.shooterRPM = prefs->GetDouble("ShootingProfile.Short.Shooter", 4250);
    shortCfg.feederRPM = prefs->GetDouble("ShootingProfile.Short.Feeder", -5000);
    shootingProfiles[ShootingProfile::kShort] = shortCfg;

    mediumCfg.shooterRPM = prefs->GetDouble("ShootingProfile.Medium.Shooter", 4600);
    mediumCfg.feederRPM = prefs->GetDouble("ShootingProfile.Medium.Feeder", -3000);
    shootingProfiles[ShootingProfile::kMedium] = mediumCfg;

    longCfg.shooterRPM = prefs->GetDouble("ShootingProfile.Long.Shooter", 4800);
    longCfg.feederRPM = prefs->GetDouble("ShootingProfile.Long.Feeder", -3000);
    shootingProfiles[ShootingProfile::kLong] = longCfg;

    autoFadeCfg.shooterRPM = prefs->GetDouble("ShootingProfile.AutoFade.Shooter", 5000);
    autoFadeCfg.feederRPM = prefs->GetDouble("ShootingProfile.AutoFade.Feeder", -2750);
    shootingProfiles[ShootingProfile::kAutoFade] = autoFadeCfg;

}

// ***************************************************************************/

void Turret::Instrument()
{
    frc::SmartDashboard::PutBoolean("ShooterEnabled", shooterEnabled);
    frc::SmartDashboard::PutNumber("Shooter Out RPM", shooterMotor->GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Amps", shooterMotor->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shooter Follower Amps", shooterMotorFollower->GetOutputCurrent());
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
