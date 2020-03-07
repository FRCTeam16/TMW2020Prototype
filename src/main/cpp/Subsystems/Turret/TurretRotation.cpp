#include "Subsystems/Turret/TurretRotation.h"
#include "Util/BSPrefs.h"

TurretRotation::TurretRotation(std::shared_ptr<VisionSystem> visionSystem) : visionSystem(visionSystem)
{
    //-------------------------------
    // Turret Limits
    //-------------------------------
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 0);
    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -715);

    //-------------------------------
    // Turret PID Dashboard Controls
    //-------------------------------
    frc::SmartDashboard::PutNumber("Turret.Setpoint", turretMotor->GetEncoder().GetPosition());
    auto prefs = BSPrefs::GetInstance();
    turretRotationPID.P = prefs->GetDouble("TurretRotation.PID.P", 0.3);
    turretRotationPID.I = prefs->GetDouble("TurretRotation.PID.I", 0);
    turretRotationPID.D = prefs->GetDouble("TurretRotation.PID.D", 0);
    turretRotationPID.F = prefs->GetDouble("TurretRotation.PID.F", 0);

    auto turretPID = turretMotor->GetPIDController();
    turretPID.SetP(turretRotationPID.P);
    turretPID.SetI(turretRotationPID.I);
    turretPID.SetD(turretRotationPID.D);
    turretPID.SetFF(turretRotationPID.F);

    //-------------------------------
    // Default initialization
    //-------------------------------
    ZeroTurretPosition();
    std::cout << "TurretRotation created\n";
}

void TurretRotation::Init()
{
    std::cout << "TurretRotation::Init =>\n";
    visionTrackingEnabled = false;
    openLoopMessage = false;

    turretSetpoint = turretMotor->GetEncoder().GetPosition();
    positionControl = true;
    std::cout << "TurretRotation::Init <=\n";
}

void TurretRotation::Run()
{
    // const double now = frc::Timer::GetFPGATimestamp();
    auto visionInfo = visionSystem->GetLastVisionInfo();
    // std::cout << "Turret(tpc=" << positionControl << ", ol=" << openLoopMessage << ", vt=" << visionTrackingEnabled << ")\n";

    if (openLoopMessage)
    {
        // Manual control always overrides
        turretMotor->Set(turretSpeed);
        openLoopMessage = false;
        positionControl = false;
    }
    else if ((visionTrackingEnabled && !positionControl) ||
            (visionTrackingEnabled && positionControl && visionInfo->hasTarget)) /* Simplify to visionTracking + hasTarget */
    {
        // We have a vision target, override turret position
        double speed = 0.0;
        if (visionInfo->hasTarget) {
            speed = -visionInfo->xSpeed;
            positionControl = false;    // kick out of position control if we see a target
        }
        turretMotor->Set(speed);
    }
    else
    {
        // We're position control mode, run until we are in position
        if (IsTurretInPosition() /*|| !positionControl*/) {
            positionControl = false;
            turretMotor->Set(0.0);
        } else {
            // UpdateTurretRotationPID();
            turretMotor->GetPIDController().SetReference(turretSetpoint, rev::ControlType::kPosition);
        }
    }
}


void TurretRotation::EnableTurretSoftLimits()
{
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
}

void TurretRotation::DisableTurretSoftLimits()
{
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    turretMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
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

void TurretRotation::SetTurretSetpointAuto(double setpoint)
{
    this->SetTurretSetpoint(turretStartPosition + setpoint);
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
    auto prefs = BSPrefs::GetInstance();
    turretStartPosition = turretMotor->GetEncoder().GetPosition();

    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, turretStartPosition + prefs->GetDouble("TurretRotation.FwdLimit", 0));
    turretMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, turretStartPosition + prefs->GetDouble("TurretRotation.RevLimit", -715));

    turretPositions.clear();
    turretPositions[Position::kRight] = turretStartPosition;
    turretPositions[Position::kBack]  = turretStartPosition + prefs->GetDouble("TurretRotation.Offset.Back", -210);
    turretPositions[Position::kLeft]  = turretStartPosition + prefs->GetDouble("TurretRotation.Offset.Left", -413);
    turretPositions[Position::kFront] = turretStartPosition + prefs->GetDouble("TurretRotation.Offset.Front", -628);
    turretPositions[Position::kGoalWallShot] = turretStartPosition + prefs->GetDouble("TurretRotation.Offset.GoalWallShot", -175);
}

// ***************************************************************************/

void TurretRotation::EnableVisionTracking()
{
    visionSystem->EnableVisionTracking();
    visionTrackingEnabled = true;
    positionControl = false;
}

void TurretRotation::DisableVisionTracking()
{
    visionSystem->DisableVisionTracking();
    visionTrackingEnabled = false;
}

void TurretRotation::ToggleVisionTracking()
{
    if (visionTrackingEnabled)
    {
        std::cout << "TurretRotation::ToggleVisionTracking -> Disable Vision Tracking\n";
        this->DisableVisionTracking();
    }
    else
    {
        std::cout << "TurretRotation::ToggleVisionTracking -> Enable Vision Tracking\n";
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
    const double physicalPosition = turretMotor->GetEncoder().GetPosition();
    frc::SmartDashboard::PutNumber("Turret Position", physicalPosition);
    frc::SmartDashboard::PutNumber("Adjusted Turret Position", physicalPosition + turretStartPosition);
    frc::SmartDashboard::PutNumber("Turret Velocity", turretMotor->GetEncoder().GetVelocity());
}

void TurretRotation::UpdateTurretRotationPID()
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