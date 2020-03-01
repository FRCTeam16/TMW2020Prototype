#include <iostream>
#include <ctre/Phoenix.h>
#include "Commands/RotateTurretToStart.h"
#include "Robot.h"




RotateTurretToStart::RotateTurretToStart(): Command() {
    SetRunWhenDisabled(false);
}

void RotateTurretToStart::Initialize() {
	std::cout << "****** RotateTurretToStart ******\n";
    Robot::turret->GetTurretRotation().SetTurretPosition(TurretRotation::Position::kRight);
	std::cout << "****** RotateTurretToStart ******\n";
	SetTimeout(1);
}

void RotateTurretToStart::Execute() {
}

bool RotateTurretToStart::IsFinished() {
    return IsTimedOut();
}

void RotateTurretToStart::End() {
}

void RotateTurretToStart::Interrupted() {
}
