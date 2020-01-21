#pragma once

#include "Subsystems/SubsystemManager.h"
#include "RobotMap.h"
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "Util/BSPrefs.h"
#include "Subsystems/Vision/VisionSystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

struct PIDConfig {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0.000015;
    double kMaxOutput = 1.0;
    double kMinOutput = -1.0;

    double kRpm1 = 0.0;
};


class Turret : public SubsystemManager {
public:
    explicit Turret(std::shared_ptr<VisionSystem> visionSystem) 
        : turretRunMode(Turret::TurretRunMode::kOpenLoop),
          visionSystem(visionSystem)
    {
        shooterMotor->SetInverted(false);
        shooterMotor->SetClosedLoopRampRate(0.5);
        shooterMotorFollower->Follow(*shooterMotor, true);
    }

    enum TurretRunMode { kOpenLoop, kClosedLoop };

    void Init() override {
        //-------------------------------
        // Shooter PID Dashboard Controls
        //-------------------------------
        frc::SmartDashboard::PutNumber("#1 P Gain", shooterPIDConfig.kP);
        frc::SmartDashboard::PutNumber("#1 I Gain", shooterPIDConfig.kI);
        frc::SmartDashboard::PutNumber("#1 D Gain", shooterPIDConfig.kD);
        frc::SmartDashboard::PutNumber("#1 I Zone", shooterPIDConfig.kIz);
        frc::SmartDashboard::PutNumber("#1 Feed Forward", shooterPIDConfig.kFF);
        frc::SmartDashboard::PutNumber("#1 Max Output", shooterPIDConfig.kMaxOutput);
        frc::SmartDashboard::PutNumber("#1 Min Output", shooterPIDConfig.kMinOutput);
        frc::SmartDashboard::PutNumber("#1 RPM", shooterPIDConfig.kRpm1);

        frc::SmartDashboard::PutNumber("SetPoint1", 0);
    }

    void Run() override {
        //----------------
        // Turret Control
        //----------------
        if (Turret::TurretRunMode::kOpenLoop == turretRunMode) {
            // std::cout << "Turret Speed: " << turretSpeed << "\n";
            turretMotor->Set(turretSpeed);
        } else {
            // Closed Loop control
            double speed = 0.0;
            auto visionInfo = visionSystem->GetLastVisionInfo();
            frc::SmartDashboard::PutBoolean("Has Target", visionInfo->hasTarget);
            frc::SmartDashboard::PutBoolean("Vision xSpeed", visionInfo->xSpeed);
            if (visionInfo->hasTarget) {
                speed = -visionInfo->xSpeed;
            }
            std::cout << "Vision Turret Speed: " << speed << "\n";
            turretMotor->Set(speed);  // Zero until we have closed loop control
        }
        frc::SmartDashboard::PutNumber("Turret Position", turretMotor->GetEncoder().GetPosition());
        frc::SmartDashboard::PutNumber("Turret Velocity", turretMotor->GetEncoder().GetVelocity());

        //----------------
        // Shooter Control
        //----------------
        frc::SmartDashboard::PutBoolean("ShooterEnabled", shooterEnabled);

        UpdateShooterPID();
        rev::CANPIDController shooterPIDController = shooterMotor->GetPIDController();
        double shooterRPM = 0.0;
        if (shooterEnabled) {
            shooterRPM = shooterPIDConfig.kRpm1;
        }
        frc::SmartDashboard::PutNumber("Velocity1", shooterMotor->GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("ShooterSetpoint", shooterRPM);
        shooterPIDController.SetReference(shooterRPM, rev::ControlType::kVelocity);
    }

	// override void Instrument() {}

    void SetTurretSpeed(double _speed) {
        turretRunMode = Turret::TurretRunMode::kOpenLoop;
        turretSpeed = _speed;
    }

    void UseVisionTracking() {
        turretRunMode = Turret::TurretRunMode::kClosedLoop;
    }

    void SetShooterEnabled(bool _enabled) {
        shooterEnabled = _enabled;
    }

    void ToggleShooterEnabled() {
        shooterEnabled = !shooterEnabled;
    }

private:
    std::shared_ptr<rev::CANSparkMax> turretMotor = RobotMap::turretMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotor = RobotMap::shooterMotor;
    std::shared_ptr<rev::CANSparkMax> shooterMotorFollower = RobotMap::shooterMotorFollower;
    std::shared_ptr<VisionSystem> visionSystem;

    Turret::TurretRunMode turretRunMode;
    double turretSpeed = 0.0;

    PIDConfig shooterPIDConfig;
    bool shooterEnabled = false;

    void UpdateShooterPID() {
        // read PID coefficients from SmartDashboard
        double p = frc::SmartDashboard::GetNumber("#1 P Gain", 0);
        double i = frc::SmartDashboard::GetNumber("#1 I Gain", 0);
        double d = frc::SmartDashboard::GetNumber("#1 D Gain", 0);
        double iz = frc::SmartDashboard::GetNumber("#1 I Zone", 0);
        double ff = frc::SmartDashboard::GetNumber("#1 Feed Forward", 0);
        double max = frc::SmartDashboard::GetNumber("#1 Max Output", 0);
        double min = frc::SmartDashboard::GetNumber("#1 Min Output", 0);
        double rpm1 = frc::SmartDashboard::GetNumber("#1 RPM", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        auto shooterPIDController = shooterMotor->GetPIDController();
        if((p != shooterPIDConfig.kP)) { shooterPIDController.SetP(p); shooterPIDConfig.kP = p; }
        if((i != shooterPIDConfig.kI)) { shooterPIDController.SetI(i); shooterPIDConfig.kI = i; }
        if((d != shooterPIDConfig.kD)) { shooterPIDController.SetD(d); shooterPIDConfig.kD = d; }
        if((iz != shooterPIDConfig.kIz)) { shooterPIDController.SetIZone(iz); shooterPIDConfig.kIz = iz; }
        if((ff != shooterPIDConfig.kFF)) { shooterPIDController.SetFF(ff); shooterPIDConfig.kFF = ff; }
        if((max != shooterPIDConfig.kMaxOutput) || (min != shooterPIDConfig.kMinOutput)) { 
            shooterPIDController.SetOutputRange(min, max); 
            shooterPIDConfig.kMinOutput = min; 
            shooterPIDConfig.kMaxOutput = max; 
        }
        if((rpm1 != shooterPIDConfig.kRpm1)) { 
            shooterPIDConfig.kRpm1 = rpm1;
        }
    }
};