#pragma once

#include "frc/PIDSource.h"
#include "Subsystems/Drive/SwerveWheel.h"

class SingleWheelEncoderPIDSource : public frc::PIDSource {
public:
  explicit SingleWheelEncoderPIDSource(shared_ptr<SwerveWheel> wheel)
    : wheel(wheel) {
      SetInitialEncoderValue();
  }

  double PIDGet() override {
    return wheel->GetDriveEncoderPosition() - startEncoderPosition;
  }

  void SetInitialEncoderValue() {
    startEncoderPosition = wheel->GetDriveEncoderPosition();
  }

private:
  shared_ptr<SwerveWheel> wheel;
  double startEncoderPosition = 0.0;
};
