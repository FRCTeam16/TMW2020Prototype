/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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

  static std::shared_ptr<BSGyro> gyro;

  static std::shared_ptr<Compressor> compressor;
};
