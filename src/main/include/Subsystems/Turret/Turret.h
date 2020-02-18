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


class Turret : public SubsystemManager {
public:
    explicit Turret(std::shared_ptr<VisionSystem> visionSystem);

    void Init() override;
    void Run() override;
	void Instrument() override;

    void SetOpenLoopTurretSpeed(double _speed);
    void OpenLoopHaltTurret();
    void SetTurretSetpoint(double setpoint);
    bool IsTurretInPosition();
    void EnableTurretPositionControl(bool control);     // FIXME: Need better state management

    void EnableVisionTracking();
    void DisableVisionTracking();
    void ToggleVisionTracking();
    bool IsVisionTracking();

    void SetShooterEnabled(bool _enabled);
    void ToggleShooterEnabled();

    void StartFeeder(bool reverse = false);
    void StopFeeder();
    void PreloadBall();

    void SetLidToLongShot();
    void SetLidToShortShot();

    

private:
    std::shared_ptr<rev::CANSparkMax> turretMotor = RobotMap::turretMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotor = RobotMap::shooterMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotorFollower = RobotMap::shooterMotorFollower;
    std::shared_ptr<rev::CANSparkMax> feederMotor = RobotMap::feederMotor;
    std::shared_ptr<frc::Solenoid> lidTop = RobotMap::lidTop;
    std::shared_ptr<VisionSystem> visionSystem;

    bool openLoopMessage = false;
    bool visionTrackingEnabled = false;

    double turretSetpoint = 0.0;
    double turretSpeed = 0.0;
    bool turretPositionControl = false;

    PIDConfig shooterPIDConfig;
    bool shooterEnabled = false;
    double visionTargetAcquiredTime = -1.0;
    const double kVisionTargetAcquiredMinWait = 0.5;

    bool feederEnabled = false;
    bool feederReversed = false;

    bool preloadFeederRunning = false;
    double preloadFeederStarted = 0.0;

    bool lidTopShort = false;
    bool lidTopMessageSent = true;

    void UpdateShooterPID();
    void UpdateTurretPID();
    void UpdateFeederPID();
};