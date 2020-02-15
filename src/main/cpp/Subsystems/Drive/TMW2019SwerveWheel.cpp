/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Drive/TMW2019SwerveWheel.h"
#include "Util/BSPrefs.h"
#include <iostream>
#include "rev/CANError.h"
#include "Util/BSPrefs.h"
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>

void TMW2019SwerveWheel::InitializeSteering() {
    assert(steerMotor.get() != nullptr);
    steerMotor->SetNeutralMode(NeutralMode::Coast);
    steerMotor->Set(ControlMode::Position, 0);
    steerMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
    steerMotor->SetInverted(false);
    steerMotor->Config_kP(0, kSteerP, 0);
    steerMotor->ConfigPeakOutputForward(0.50, 0);
    steerMotor->ConfigPeakOutputReverse(-0.50, 0);
    steerMotor->ConfigPeakCurrentLimit(0);
    steerMotor->ConfigContinuousCurrentLimit(BSPrefs::GetInstance()->GetDouble("Steer.ContinuousCurrentLimit", 20.0));
}

void TMW2019SwerveWheel::InitializeDrivePID() {
    assert(driveMotor.get() != nullptr);
    BSPrefs *prefs = BSPrefs::GetInstance();
    const double driveP = prefs->GetDouble("DriveP", 0.0);
	const double driveI = prefs->GetDouble("DriveI", 0.0);
	const double driveD = prefs->GetDouble("DriveD", 0.0);
	const double driveF = prefs->GetDouble("DriveF", 0.0);
	const double driveIZone = prefs->GetDouble("DriveIZone", 0.0);

    // driveMotor->Follow(rev::CANSparkMax::kFollowerDisabled, 0);
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    driveMotor->Set(0.0);
    isOpenLoop = true;

    rev::CANPIDController pid = driveMotor->GetPIDController();
    pid.SetP(driveP);    
    pid.SetI(driveI);
    pid.SetD(driveD);
    pid.SetFF(driveF);
    pid.SetIZone(driveIZone);
    pid.SetOutputRange(-1.0, 1.0);
}

bool TMW2019SwerveWheel::IsOpenLoopDrive() {
    return isOpenLoop;
}

double TMW2019SwerveWheel::GetDriveEncoderPosition() {
    return driveMotor->GetEncoder().GetPosition();
}

double TMW2019SwerveWheel::GetDriveVelocity() {
    return driveMotor->GetEncoder().GetVelocity();
}

void TMW2019SwerveWheel::ZeroDriveEncoder() {
    // talon motor->SetSelectedSensorPosition(0, 0, 0);
    //driveMotor->GetPIDController().  
    std::cout << "[ERROR] Request to Zero Drive Encoders, but SparkMax does not support that operation!\n";
}

double TMW2019SwerveWheel::GetDriveOutputCurrent() {
    return driveMotor->GetOutputCurrent();
}

void TMW2019SwerveWheel::InitTeleop() {
    steerMotor->Config_kP(0, kSteerP, 0);
}

void TMW2019SwerveWheel::InitAuto() {
    steerMotor->Config_kP(0, 3.0, 0);
}

void TMW2019SwerveWheel::UseOpenLoopDrive(double speed) {
    // driveMotor->Set(ControlMode::PercentOutput, 0.0);
    if (speed < driveMinOutput) { speed = driveMinOutput; }
    if (speed > driveMaxOutput) { speed = driveMaxOutput; }
    
    frc::SmartDashboard::PutNumber(name + " Drive Speed", speed);
    // std::cout << "TMW2019SwerveWheel " << name << " OpenLoop: " << speed << "\n";
    // std::cout << "TMW2019SwerveWheel " << name << " Follower? " << driveMotor->IsFollower() << "\n";

    driveMotor->Set(speed);
    isOpenLoop = true;
}

void TMW2019SwerveWheel::UseClosedLoopDrive(double value, double maxOutput) {
    // driveMotor->Set(ControlMode::Velocity, 0);
    std::cout << "TMW2019SwerveWheel::UseClosedLoopDrive(" << value << ", maxOutput = " << maxOutput << ")\n";

    BSPrefs *prefs = BSPrefs::GetInstance();
    const double driveP = prefs->GetDouble("DriveP", 0.4);
	const double driveI = prefs->GetDouble("DriveI", 0.0);
	const double driveD = prefs->GetDouble("DriveD", 0.0);
	const double driveF = prefs->GetDouble("DriveF", 0.0);
	const double driveIZone = prefs->GetDouble("DriveIZone", 0.0);
    const double driveRamp = prefs->GetDouble("DriveRampRate", 0.0);
    driveMotor->SetClosedLoopRampRate(driveRamp);


    rev::CANPIDController pid = driveMotor->GetPIDController();
    pid.SetP(driveP);    
    pid.SetI(driveI);
    pid.SetD(driveD);
    pid.SetFF(driveF);
    pid.SetIZone(driveIZone);
    pid.SetOutputRange(-maxOutput, maxOutput);

    pid.SetReference(value, rev::ControlType::kPosition);
    isOpenLoop = false;
}

void TMW2019SwerveWheel::UseClosedLoopSpeedDrive(double velocity) {
    BSPrefs *prefs = BSPrefs::GetInstance();
    const double driveP = prefs->GetDouble("DriveP", 0.4);
	const double driveI = prefs->GetDouble("DriveI", 0.0);
	const double driveD = prefs->GetDouble("DriveD", 0.0);
	const double driveF = prefs->GetDouble("DriveF", 0.0);
	const double driveIZone = prefs->GetDouble("DriveIZone", 0.0);

    rev::CANPIDController pid = driveMotor->GetPIDController();
    pid.SetP(driveP);    
    pid.SetI(driveI);
    pid.SetD(driveD);
    pid.SetFF(driveF);
    pid.SetIZone(driveIZone);

    pid.SetOutputRange(-1.0, 1.0);
    // velocity needs to be in RPMs
    pid.SetReference(velocity, rev::ControlType::kVelocity);
    isOpenLoop = false;
}

std::shared_ptr<rev::CANSparkMax> TMW2019SwerveWheel::GetDriveMotor() {
    return driveMotor;
}

bool TMW2019SwerveWheel::HasCANError() {
    // TOOD: Is this the best way to get CANError info?  Seems like others
    // require sending configuration information
    rev::CANError error = driveMotor->ClearFaults();
    return error != rev::CANError::kOk;
}

void TMW2019SwerveWheel::SetDriveSoftMinMaxOutput(double minOutput, double maxOutput) {
    driveMinOutput = minOutput;
    driveMaxOutput = maxOutput;
}


// ****************************************************************************//

double TMW2019SwerveWheel::GetSteerEncoderPositionInDegrees() {
    int currentPosition = steerMotor->GetSelectedSensorPosition(0);
	int currentPositionEncoderUnits = currentPosition % 4096;
	double positionInDegrees = (currentPositionEncoderUnits / 4096.0 * 360.0);

	if (positionInDegrees < -180.0) {
		positionInDegrees += 360.0;
	} else if (positionInDegrees > 180.0) {
		positionInDegrees -= 360.0;
	}
	return positionInDegrees;
}

void TMW2019SwerveWheel::SetSteerEncoderSetpoint(double setpoint, double offset, int &inv) {
    double baseCurrentPosition = steerMotor->GetSelectedSensorPosition(0);
    double currentPosition = baseCurrentPosition / 4096.0;
	double setpointRotations = (setpoint + offset) / 360.0;

    // double spRotations = 0.0;
    // double spDiff = modf(setpointRotations, &spRotations);
	
    // double cpRotations = 0.0;
    // double cpDiff = modf(currentPosition, &cpRotations);
    // double initCpDiff = cpDiff;

    // double spRotations = 0.0;
    // double spDiff = modf(setpointRotations, &spRotations);

    // // Unify windings/signs
    // double rawDiff = setpointRotations - cpDiff;
    
	// double diff = spDiff - cpDiff;
    double wholeRotations = 0.0;
    // double diff = modf(setpointRotations - cpDiff, &wholeRotations);
    double diff = modf(setpointRotations - currentPosition, &wholeRotations);

    // Normalize difference into the nearest half of circle (positive or negative)
    if (fabs(diff) > 0.5) {
        if (diff < 0 ) {
            diff = 1.0 + diff;
        } else {
            diff = -1.0 + diff;
        }
    }

    // Invert wheel if difference > 90 degrees
	if (fabs(diff) > 0.25) {
		diff -= copysign(0.5, diff);
		inv = -1;
	} else {
		inv = 1;
	}
    
	double finalSetpoint = currentPosition + diff;
    // if (name == "FR") {
    //     std::cout << name << 
    //             ": base: " << baseCurrentPosition <<
    //             " | setpoint: " << setpoint <<
    //             " | curPos: " << currentPosition <<
    //             " | spRot: " << setpointRotations <<
    //             " | cpDiff: " << cpDiff <<
    //             " | diff: " << diff <<
    //             " | final: " << finalSetpoint << std::endl;
    // }
	steerMotor->Set(ControlMode::Position, finalSetpoint * 4096.0);

    // steerLog.Log(
    //     frc::Timer::GetFPGATimestamp(),
    //     baseCurrentPosition,
    //     setpoint,
    //     currentPosition,
    //     setpointRotations,
    //     diff,
    //     wholeRotations,
    //     finalSetpoint);
}

int TMW2019SwerveWheel::GetSteerEncoderPosition() {
    return steerMotor->GetSelectedSensorPosition(0);
}

double TMW2019SwerveWheel::GetSteerVelocity() {
    return steerMotor->GetSelectedSensorVelocity(0);
}

double TMW2019SwerveWheel::GetSteerOutputCurrent() {
    return steerMotor->GetOutputCurrent();
}

void TMW2019SwerveWheel::UseOpenLoopSteer(double speed) {
    steerMotor->Set(ControlMode::PercentOutput, speed);
}

void TMW2019SwerveWheel::UseClosedLoopSteer(double value) {
    steerMotor->Set(ControlMode::Position, value);
}

std::shared_ptr<WPI_TalonSRX> TMW2019SwerveWheel::GetSteerMotor() {
    return steerMotor;
}


void TMW2019SwerveWheel::SetDriveBrakeMode() {
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void TMW2019SwerveWheel::SetDriveCoastMode() {
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}
