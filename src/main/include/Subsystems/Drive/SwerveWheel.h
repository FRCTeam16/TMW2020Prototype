#pragma once

using namespace std;

class SwerveWheel {
public:
  virtual ~SwerveWheel() {}
  virtual void InitTeleop() = 0;
  virtual void InitAuto() = 0;
  virtual void InitializeSteering() = 0;
  virtual void InitializeDrivePID() = 0;

  virtual bool IsOpenLoopDrive() = 0;
  virtual void UseOpenLoopDrive(double speed = 0.0) = 0;
  virtual void UseClosedLoopDrive(double value = 0.0, double maxOutput = 1.0) = 0;
  virtual void UseClosedLoopSpeedDrive(double speed = 0.0) = 0;
  virtual double GetDriveEncoderPosition() = 0; 
  virtual double GetDriveVelocity() = 0;
  virtual void ZeroDriveEncoder() = 0;
  virtual double GetDriveOutputCurrent() = 0;
  virtual bool HasCANError() = 0;
  virtual void SetDriveSoftMinMaxOutput(double minOutput, double maxOutput) = 0;

  virtual void UseOpenLoopSteer(double speed = 0.0) = 0;
  virtual void UseClosedLoopSteer(double value = 0.0) = 0;
  virtual int GetSteerEncoderPosition() = 0;
  virtual double GetSteerEncoderPositionInDegrees() = 0;
  virtual void SetSteerEncoderSetpoint(double setpoint, double offset, int &inv) = 0;
  virtual double GetSteerVelocity() = 0;
  virtual double GetSteerOutputCurrent() = 0;

  virtual void SetDriveBrakeMode() = 0;
  virtual void SetDriveCoastMode() = 0;
};
