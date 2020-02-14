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

    void SetTurretSpeed(double _speed) {
        openLoopMessage = true;
        turretSpeed = _speed;
    }

    void HaltManualTurret() {
        turretSpeed = 0.0;
    }

    void EnableVisionTracking() {
        visionTrackingEnabled = true;
    }

    void DisableVisionTracking() {
        visionTrackingEnabled = false;
    }

    void ToggleVisionTracking() {
        if (visionTrackingEnabled) {
            // turn off
            visionSystem->DisableVisionTracking();
            this->DisableVisionTracking();
        } else {
            // turn on
            visionSystem->EnableVisionTracking();
            this->EnableVisionTracking();
        }
    }

    bool IsVisionTracking() {
        return visionTrackingEnabled;
    }

    void SetShooterEnabled(bool _enabled) {
        shooterEnabled = _enabled;
    }

    void ToggleShooterEnabled() {
        shooterEnabled = !shooterEnabled;
    }

    void StartFeeder(bool reverse = false){
        feederEnabled = true;
        feederReversed = reverse;
    }

    void StopFeeder(){
        feederEnabled = false;
    }

    void PreloadBall(){
        if (!preloadFeederRunning) {
            preloadFeederRunning = true;
            preloadFeederStarted = frc::Timer::GetFPGATimestamp();

        }
    }

    void SetLidToLongShot() {
        lidTopShort = false;
        lidTopMessageSent = false;
    }

    void SetLidToShortShot() {
        lidTopShort = true;
        lidTopMessageSent = false;
    }

private:
    std::shared_ptr<rev::CANSparkMax> turretMotor = RobotMap::turretMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotor = RobotMap::shooterMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotorFollower = RobotMap::shooterMotorFollower;
    std::shared_ptr<rev::CANSparkMax> feederMotor = RobotMap::feederMotor;
    std::shared_ptr<Solenoid> lidTop = RobotMap::lidTop;
    std::shared_ptr<VisionSystem> visionSystem;

    bool openLoopMessage = false;
    bool visionTrackingEnabled = false;
    double turretSpeed = 0.0;

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
};