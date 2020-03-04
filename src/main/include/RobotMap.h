#pragma once

#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/PowerDistributionPanel.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include "Gyro/BSGyro.h"

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

  static shared_ptr<rev::CANSparkMax> turretMotor;

  static shared_ptr<rev::CANSparkMax> shooterMotor;
  static shared_ptr<rev::CANSparkMax> shooterMotorFollower;
  static shared_ptr<rev::CANSparkMax> feederMotor;

  static shared_ptr<WPI_TalonFX> armMotor;
  static shared_ptr<WPI_TalonFX> armMotorFollower;


  static shared_ptr<WPI_VictorSPX> intakeMotor;
  static std::shared_ptr<BSGyro> gyro;

  static std::shared_ptr<frc::Compressor> compressor;
  static std::shared_ptr<frc::Solenoid> lidTop;
  static std::shared_ptr<frc::Solenoid> climberLeftArm;
  static std::shared_ptr<frc::Solenoid> climberRightArm;

  static std::shared_ptr<frc::PowerDistributionPanel> powerDistributionPanel;
};
