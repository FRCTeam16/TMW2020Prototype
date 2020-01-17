/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Util/SwerveWheelLog.h"

SwerveWheelLog::SwerveWheelLog(std::string name_) : name(name_)
{
    
}

void SwerveWheelLog::Log(
    double timestamp,
    double baseCurrentPosition,
    double setpoint,
    double currentPosition,
    double setpointRotations,
    double diff,
    double wholeRotations,
    double finalSetpoint) {

    if (!enabled) {
        return;
    }

    if (!headerWritten) {
        std::string filename = "/home/lvuser/steer-" + name + ".csv";
        logfile.open(filename, ios::out);
        logfile << "timestamp, baseSensor, setpoint, "
                << "currentPosition, setpointRotations, "
                << "diff, wholeRot, finalSetpoint\n";
        headerWritten = true;
    }
    
    logfile << timestamp << ","
            << baseCurrentPosition << ","
            << setpoint << ","
            << currentPosition << ","
            << setpointRotations << ","
            << diff << ","
            << wholeRotations << ","
            << finalSetpoint << "\n";
}
