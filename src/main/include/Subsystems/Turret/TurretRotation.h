#pragma once

#include <rev/CANSparkMax.h>
#include <unordered_map>
#include "Subsystems/SubsystemManager.h"
#include "Subsystems/Vision/VisionSystem.h"
#include "RobotMap.h"
#include "Util/PIDConfig.h"


class TurretRotation : public SubsystemManager {
public:
    enum Position { kLeft, kFront, kRight, kBack, kGoalWallShot };
    
    explicit TurretRotation(std::shared_ptr<VisionSystem> visionSystem);
    void Init() override;
    void Run() override;
	void Instrument() override;

    void SetOpenLoopTurretSpeed(double _speed);
    void OpenLoopHaltTurret();
    void SetTurretPosition(Position position);
    void SetTurretSetpoint(double setpoint);    // raw value
    void SetTurretSetpointAuto(double setpoint);
    bool IsTurretInPosition();
    void ZeroTurretPosition();

    void EnableTurretSoftLimits();
    void DisableTurretSoftLimits();

    void EnableVisionTracking();
    void DisableVisionTracking();
    void ToggleVisionTracking();
    bool IsVisionTracking();

private:
    std::shared_ptr<rev::CANSparkMax> turretMotor = RobotMap::turretMotor;
    std::shared_ptr<VisionSystem> visionSystem;

    PIDConfig turretRotationPID;
    void UpdateTurretRotationPID();

    bool visionTrackingEnabled = false;
    double visionTargetAcquiredTime = -1.0;
    const double kVisionTargetAcquiredMinWait = 0.5;

    std::unordered_map<Position, double> turretPositions;
    double turretStartPosition = 0.0;
    double turretSetpoint = 0.0;
    double turretBackPosition = 0.0;
    double turretFrontPosition = 0.0;
    double turretSpeed = 0.0;

    bool openLoopMessage = false;
    bool positionControl = false;
};