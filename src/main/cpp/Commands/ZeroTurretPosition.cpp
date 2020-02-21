#include <iostream>
#include <ctre/Phoenix.h>
#include "Commands/ZeroTurretPosition.h"
#include "Robot.h"

ZeroTurretPosition::ZeroTurretPosition() : Command()
{
    SetRunWhenDisabled(true);
}

// Called just before this Command runs the first time
void ZeroTurretPosition::Initialize()
{
    std::cout << "****** ZERO TURRET POSITION ******\n";
    Robot::turret->GetTurretRotation().ZeroTurretPosition();
    std::cout << "****** ZERO TURRET POSITION ******\n";
    SetTimeout(1);
}

// Called repeatedly when this Command is scheduled to run
void ZeroTurretPosition::Execute()
{
}

// Make this return true when this Command no longer needs to run execute()
bool ZeroTurretPosition::IsFinished()
{
    return IsTimedOut();
}

// Called once after isFinished returns true
void ZeroTurretPosition::End()
{
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ZeroTurretPosition::Interrupted()
{
}
