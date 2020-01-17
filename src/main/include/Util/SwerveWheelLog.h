/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <iostream>
#include <fstream>

using namespace std;

class SwerveWheelLog {
 public:
  SwerveWheelLog(std::string name);
  void Log(
    double timestamp,
    double baseCurrentPosition,
    double setpoint,
    double currentPosition,
    double setpointRotations,
    double diff,
    double wholeRotations,
    double finalSetpoint);

  void Enable() { enabled = true; }
  void Disable() { enabled = false; }

 private:
  const std::string name;
  bool enabled = true;
  bool headerWritten = false;
  ofstream logfile;
};
