#include "Subsystems/Turret/TurretRotation.h"

TurretRotation::TurretRotation(std::shared_ptr<VisionSystem> visionSystem) : visionSystem(visionSystem)
{

    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 0);
    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -715);
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);

    //-------------------------------
    // Turret PID Dashboard Controls
    //-------------------------------
    frc::SmartDashboard::PutNumber("Turret.Setpoint", turretMotor->GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Turret.PID.P", 2.0);
    frc::SmartDashboard::PutNumber("Turret.PID.I", 0);
    frc::SmartDashboard::PutNumber("Turret.PID.D", 0);
    frc::SmartDashboard::PutNumber("Turret.RPM.Long", 5000);
    frc::SmartDashboard::PutNumber("Turret.RPM.Short", 3800);

    frc::SmartDashboard::PutNumber("Turret.Pos.Front", -210);
    frc::SmartDashboard::PutNumber("Turret.Pos.Right", -413);
    frc::SmartDashboard::PutNumber("Turret.Pos.Back", -628);

    std::cout << "TurretRotation created\n";
}

void TurretRotation::Init()
{
    visionTrackingEnabled = false;
    openLoopMessage = false;

    turretStartPosition = turretMotor->GetEncoder().GetPosition();
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
    double target = turretMotor->GetEncoder().GetPosition(); // default
    switch (position)
    {
    case Position::kBack:
        target = -210;
        break;
    case Position::kLeft:
        target = 0;
        break;
    case Position::kRight:
        target = -413;
        break;
    case Position::kFront:
        target = -628;
        break;
    default:
        break;
    }
    this->SetTurretSetpoint(target);
}

void TurretRotation::SetTurretSetpoint(double setpoint)
{
    std::cout << "*************** TURRET::SetTurretSetpoint(" << setpoint << ")\n";
    turretSetpoint = setpoint;
    positionControl = true;
    frc::SmartDashboard::PutNumber("Turret.Setpoint", turretSetpoint);
}

bool TurretRotation::IsTurretInPosition()
{
    double error = fabs(turretSetpoint - turretMotor->GetEncoder().GetPosition());
    return (error < 1);
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
    double p = frc::SmartDashboard::GetNumber("Turret.PID.P", 2.0);
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