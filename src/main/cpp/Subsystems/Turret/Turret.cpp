#include "Subsystems/Turret/Turret.h"

Turret::Turret(std::shared_ptr<VisionSystem> visionSystem)
    : visionSystem(visionSystem),
      turretRunMode(Turret::TurretRunMode::kOpenLoop)
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
    shooterPIDConfig.kRpm1 = 4300;

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

void Turret::Run() 
{
    const double now = Timer::GetFPGATimestamp();
    auto visionInfo = visionSystem->GetLastVisionInfo();
    
    //----------------
    // Turret Control
    //----------------
    if (Turret::TurretRunMode::kOpenLoop == turretRunMode) {
        // std::cout << "Turret Speed: " << turretSpeed << "\n";
        turretMotor->Set(-turretSpeed);
    } else {
        // Closed Loop control
        double speed = 0.0;
        
        frc::SmartDashboard::PutBoolean("Viz Target?", visionInfo->hasTarget);
        if (visionInfo->hasTarget) {
            if (visionTargetAcquiredTime < 0) {
                visionTargetAcquiredTime = Timer::GetFPGATimestamp();
            }
            speed = -visionInfo->xSpeed;
        } else {
            visionTargetAcquiredTime = -1.0;
        }
        turretMotor->Set(speed);  // Zero until we have closed loop control
    }

    //----------------
    // Feeder Control
    //----------------
    double feederSpeed = 0.0;
    if (feederEnabled) {
        const bool tracking = visionSystem->IsVisionTrackingEnabled();
        const double timeAcquired = (now - visionTargetAcquiredTime) > kVisionTargetAcquiredMinWait;
        // if ((tracking && timeAcquired) || !tracking) {
        //     feederSpeed = frc::SmartDashboard::GetNumber("FeederSpeed", -0.8);
        // }
        feederSpeed = frc::SmartDashboard::GetNumber("FeederSpeed", -0.8);
        if (feederReversed) {
            feederSpeed = -feederSpeed;
        }
    }
    feederMotor->Set(feederSpeed);
    
    //----------------
    // Shooter Control
    //----------------
    UpdateShooterPID();
    if (shooterEnabled) {
        rev::CANPIDController shooterPIDController = shooterMotor->GetPIDController();
        double shooterRPM = shooterPIDConfig.kRpm1;
        frc::SmartDashboard::PutNumber("Velocity1", shooterMotor->GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("ShooterSetpoint", shooterRPM);
        shooterPIDController.SetReference(shooterRPM, rev::ControlType::kVelocity);
    } else {
        // Go open loop
        shooterMotor->Set(0.0);
    }
}

void Turret::Instrument()
{
    frc::SmartDashboard::PutNumber("Turret Position", turretMotor->GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Turret Velocity", turretMotor->GetEncoder().GetVelocity());
    frc::SmartDashboard::PutBoolean("ShooterEnabled", shooterEnabled); 
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


