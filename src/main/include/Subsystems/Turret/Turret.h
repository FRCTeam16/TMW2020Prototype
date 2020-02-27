#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "Subsystems/SubsystemManager.h"
#include "Subsystems/Vision/VisionSystem.h"
#include "RobotMap.h"
#include "Util/BSPrefs.h"
#include <frc/Timer.h>
#include <frc/Solenoid.h>
#include <unordered_map>

#include "Subsystems/Turret/TurretRotation.h"

enum ShootingProfile { kShort, kMedium, kLong };

struct ShootingProfileConfig {
    double shooterRPM = 0;
    double feederRPM = 0;
};

struct FeederConfig {
    double preloadTime = 0.4;
    double preloadSpeed = -0.2;
    double P = 0.00001;
    double I = 0.0;
    double D = 0.0;
    double F = 0.000185;
};

struct ShooterConfig {
    double P = 0.0004;
    double I = 0.0;
    double D = 0.0;
    double F = 0.000173;
};

class Turret : public SubsystemManager {
public:
    explicit Turret(std::shared_ptr<VisionSystem> visionSystem);

    void Init() override;
    void Run() override;
	void Instrument() override;

    TurretRotation& GetTurretRotation();

    void SetShooterEnabled(bool _enabled);
    void ToggleShooterEnabled();

    void StartFeeder(bool reverse = false);
    void StopFeeder();
    void PreloadBall();

    void SetFeederAndShooterReversed(bool reversed);

    void SetLidToLongShot();
    void SetLidToShortShot();

    void SetShootingProfile(ShootingProfile profile);

    

private:
    std::shared_ptr<rev::CANSparkMax> shooterMotor = RobotMap::shooterMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotorFollower = RobotMap::shooterMotorFollower;
    std::shared_ptr<rev::CANSparkMax> feederMotor = RobotMap::feederMotor;
    std::shared_ptr<frc::Solenoid> lidTop = RobotMap::lidTop;
    std::shared_ptr<VisionSystem> visionSystem;
    TurretRotation turretRotation;

    unordered_map<ShootingProfile, ShootingProfileConfig> shootingProfiles;
    FeederConfig feederConfig;
    ShooterConfig shooterConfig;
    bool shooterEnabled = false;

    bool feederEnabled = false;
    bool feederReversed = false;

    bool feederAndShooterReversed = false;  // try to dump balls backwards to get out of jam

    bool preloadFeederRunning = false;
    double preloadFeederStarted = 0.0;

    bool lidTopShort = false;
    bool lidTopMessageSent = true;

    void InitShootingProfiles();

    void UpdateShooterPID();
    void UpdateTurretPID();
    void UpdateFeederPID();
};