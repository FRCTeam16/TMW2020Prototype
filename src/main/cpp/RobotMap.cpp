/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotMap.h"

  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseFrontLeftDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontLeftSteer;
  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseFrontRightDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontRightSteer;
  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseRearLeftDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearLeftSteer;
  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseRearRightDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearRightSteer;

  shared_ptr<rev::CANSparkMax> RobotMap::turretMotor;
  shared_ptr<rev::CANSparkMax> RobotMap::shooterMotor;
  shared_ptr<rev::CANSparkMax> RobotMap::shooterMotorFollower;

  shared_ptr<WPI_TalonSRX> RobotMap::gyroTalon;
  std::shared_ptr<BSGyro> RobotMap::gyro;
  
  std::shared_ptr<Compressor> RobotMap::compressor;


RobotMap::RobotMap() {
  driveBaseFrontLeftDrive.reset(new rev::CANSparkMax{1, rev::CANSparkMax::MotorType::kBrushless});
  driveBaseFrontRightDrive.reset(new rev::CANSparkMax{3, rev::CANSparkMax::MotorType::kBrushless});
  driveBaseRearLeftDrive.reset(new rev::CANSparkMax{5, rev::CANSparkMax::MotorType::kBrushless});
  driveBaseRearRightDrive.reset(new rev::CANSparkMax{7, rev::CANSparkMax::MotorType::kBrushless});

  driveBaseFrontLeftSteer.reset(new WPI_TalonSRX{2});
  driveBaseFrontRightSteer.reset(new WPI_TalonSRX{4});
  driveBaseRearLeftSteer.reset(new WPI_TalonSRX{6});
  driveBaseRearRightSteer.reset(new WPI_TalonSRX{8});

  turretMotor.reset(new rev::CANSparkMax{11, rev::CANSparkMax::MotorType::kBrushless});
  shooterMotor.reset(new rev::CANSparkMax{61, rev::CANSparkMax::MotorType::kBrushless});
  shooterMotorFollower.reset(new rev::CANSparkMax{62, rev::CANSparkMax::MotorType::kBrushless});

  gyroTalon.reset(new WPI_TalonSRX{10});
  gyro.reset(new BSGyro(gyroTalon.get())); 

  compressor.reset(new Compressor{0});
  compressor->SetClosedLoopControl(true);
}
