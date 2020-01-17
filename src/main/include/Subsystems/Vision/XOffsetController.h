#pragma once

#include "frc/PIDSource.h"
#include "frc/PIDOutput.h"
#include "Subsystems/Vision/Limelight.h"

class XOffsetController : public frc::PIDSource, public frc::PIDOutput {  
public:
    explicit XOffsetController(std::shared_ptr<Limelight> limelight)
    :  limelight(limelight) {}
  
  double PIDGet() override {
    double xOffset = limelight->GetScene().xOffset;
    return xOffset;
  }

  void PIDWrite(double value) override {
    pidValue = value;
  }

  double GetValue() const {
    return pidValue;
  }

private:
  std::shared_ptr<Limelight> limelight;
  double pidValue = 0.0;
};
