#include "Subsystems/Turret/TurretRotation.h"

TurretRotation::TurretRotation(std::shared_ptr<VisionSystem> visionSystem) : visionSystem(visionSystem)
{
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 0);
    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -715);


    //-------------------------------
    // Turret PID Dashboard Controls
    //-------------------------------
    frc::SmartDashboard::PutNumber("Turret.Setpoint", turretMotor->GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Turret.PID.P", 0.3);
    frc::SmartDashboard::PutNumber("Turret.PID.I", 0);
    frc::SmartDashboard::PutNumber("Turret.PID.D", 0);
    frc::SmartDashboard::PutNumber("Turret.RPM.Long", 5000);
    frc::SmartDashboard::PutNumber("Turret.RPM.Short", 3800);
    std::cout << "TurretRotation created\n";
}

void TurretRotation::Init()
{
    visionTrackingEnabled = false;
    openLoopMessage = false;

    ZeroTurretPosition();
    turretSetpoint = turretStartPosition;
    positionControl = true;
}

void TurretRotation::Run()
{
    const double now = frc::Timer::GetFPGATimestamp();
    auto visionInfo = visionSystem->GetLastVisionInfo();
    // std::cout << "Turret(tpc=" << positionControl << ", ol=" << openLoopMessage << ", vt=" << visionTrackingEnabled << ")\n";

    if (openLoopMessage)
    {
        // Manual control always overrides
        turretMotor->Set(turretSpeed);
        openLoopMessage = false;
        positionControl = false;
    }
    else if (visionTrackingEnabled && visionInfo->hasTarget)
    {
        // We have a vision target, override turret position
        double speed = -visionInfo->xSpeed;
        turretMotor->Set(speed);
    }
    else
    {
        // We're position control mode, run until we are in position
        if (IsTurretInPosition() /*|| !positionControl*/) {
            positionControl = false;
            turretMotor->Set(0.0);
        } else {
            UpdateTurretPID();
            turretMotor->GetPIDController().SetReference(turretSetpoint, rev::ControlType::kPosition);
        }
    }
}

// ***************************************************************************/

void TurretRotation::SetOpenLoopTurretSpeed(double _speed)
{
    openLoopMessage = true;
    turretSpeed = _speed;
}

void TurretRotation::OpenLoopHaltTurret()
{
    // If we just sent open loop information, send one more to halt the turret
    if (turretSpeed != 0.0)
    {
        openLoopMessage = true;
        SetTurretSetpoint(turretMotor->GetEncoder().GetPosition());
    }
    turretSpeed = 0.0;
}

void TurretRotation::SetTurretPosition(Position position)
{
    auto iter = turretPositions.find(position);
    if (iter == turretPositions.end()) {
        std::cout << "*** ERROR: TurretRotation::SetTurretPosition unable to "
                  << "locate requested position: " << position << "\n";
    } else {
        this->SetTurretSetpoint(iter->second);    
    }
}

void TurretRotation::SetTurretSetpoint(double setpoint)
{
    std::cout << "TurretRotation::SetTurretSetpoint(" << setpoint << ")\n";
    turretSetpoint = setpoint;
    positionControl = true;
    frc::SmartDashboard::PutNumber("Turret.Setpoint", turretSetpoint);
}

bool TurretRotation::IsTurretInPosition()
{
    double error = fabs(turretSetpoint - turretMotor->GetEncoder().GetPosition());
    return (error < 1); //FIXME: Lookup Allowed Error
}

void TurretRotation::ZeroTurretPosition()
{
    turretStartPosition = turretMotor->GetEncoder().GetPosition();
    turretPositions.clear();
    turretPositions[Position::kRight] = turretStartPosition;
    turretPositions[Position::kBack]  = turretStartPosition - 210;
    turretPositions[Position::kLeft]  = turretStartPosition - 413;
    turretPositions[Position::kFront] = turretStartPosition - 628;
    turretPositions[Position::kGoalWallShot] = turretStartPosition - 100;
}

// ***************************************************************************/

void TurretRotation::EnableVisionTracking()
{
    visionTrackingEnabled = true;
    positionControl = false;
}

void TurretRotation::DisableVisionTracking()
{
    visionTrackingEnabled = false;
}

void TurretRotation::ToggleVisionTracking()
{
    if (visionTrackingEnabled)
    {
        // turn off
        visionSystem->DisableVisionTracking();
        this->DisableVisionTracking();
    }
    else
    {
        // turn on
        visionSystem->EnableVisionTracking();
        this->EnableVisionTracking();
    }
}

bool TurretRotation::IsVisionTracking()
{
    return visionTrackingEnabled;
}

// ***************************************************************************/

void TurretRotation::Instrument()
{
    frc::SmartDashboard::PutNumber("Turret Position", turretMotor->GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Turret Velocity", turretMotor->GetEncoder().GetVelocity());
}

void TurretRotation::UpdateTurretPID()
{
    double setpoint = frc::SmartDashboard::GetNumber("Turret.Setpoint", turretSetpoint);
    double p = frc::SmartDashboard::GetNumber("Turret.PID.P", 0.3);
    double i = frc::SmartDashboard::GetNumber("Turret.PID.I", 0);
    double d = frc::SmartDashboard::GetNumber("Turret.PID.D", 0);
    double f = frc::SmartDashboard::GetNumber("Turret.PID.F", 0);

    if (setpoint != turretSetpoint)
    {
        this->SetTurretSetpoint(setpoint);
    }

    auto turretPID = turretMotor->GetPIDController();
    turretPID.SetP(p);
    turretPID.SetI(i);
    turretPID.SetD(d);
    turretPID.SetFF(f);
}