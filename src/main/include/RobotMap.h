#pragma once

#include <frc/Compressor.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "Util/BSGyro.h"

using namespace std;

class RobotMap {
 public:
  RobotMap();
  static shared_ptr<rev::CANSparkMax> driveBaseFrontLeftDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseFrontLeftSteer;

  static shared_ptr<rev::CANSparkMax> driveBaseFrontRightDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseFrontRightSteer;

  static shared_ptr<rev::CANSparkMax> driveBaseRearLeftDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseRearLeftSteer;
  
  static shared_ptr<rev::CANSparkMax> driveBaseRearRightDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseRearRightSteer;

  static shared_ptr<WPI_TalonSRX> turretMotor;

  static shared_ptr<rev::CANSparkMax> shooterMotor;
  static shared_ptr<rev::CANSparkMax> shooterMotorFollower;

  static shared_ptr<WPI_TalonSRX> gyroTalon;
  static std::shared_ptr<BSGyro> gyro;

  static std::shared_ptr<Compressor> compressor;
};
