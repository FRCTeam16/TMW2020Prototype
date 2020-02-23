#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "Subsystems/SubsystemManager.h"
#include "Subsystems/Vision/VisionSystem.h"
#include "RobotMap.h"
#include "Util/BSPrefs.h"
#include "Util/PIDConfig.h"
#include <frc/Timer.h>
#include <frc/Solenoid.h>

#include "Subsystems/Turret/TurretRotation.h"

class Turret : public SubsystemManager {
public:
    explicit Turret(std::shared_ptr<VisionSystem> visionSystem);

    void Init() override;
    void Run() override;
	void Instrument() override;

    TurretRotation& GetTurretRotation();    // bad practice

    void SetShooterEnabled(bool _enabled);
    void ToggleShooterEnabled();

    void StartFeeder(bool reverse = false);
    void StopFeeder();
    void PreloadBall();

    void SetFeederAndShooterReversed(bool reversed);

    void SetLidToLongShot();
    void SetLidToShortShot();

    

private:
    std::shared_ptr<rev::CANSparkMax> shooterMotor = RobotMap::shooterMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotorFollower = RobotMap::shooterMotorFollower;
    std::shared_ptr<rev::CANSparkMax> feederMotor = RobotMap::feederMotor;
    std::shared_ptr<frc::Solenoid> lidTop = RobotMap::lidTop;
    std::shared_ptr<VisionSystem> visionSystem;
    TurretRotation turretRotation;


    PIDConfig shooterPIDConfig;
    bool shooterEnabled = false;

    bool feederEnabled = false;
    bool feederReversed = false;

    bool feederAndShooterReversed = false;  // try to dump balls backwards to get out of jam

    bool preloadFeederRunning = false;
    double preloadFeederStarted = 0.0;

    bool lidTopShort = false;
    bool lidTopMessageSent = true;

    void UpdateShooterPID();
    void UpdateTurretPID();
    void UpdateFeederPID();
};