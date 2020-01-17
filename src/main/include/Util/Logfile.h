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

/**
 * Utility class for emitting logfiles.  Will automatically write the header
 * at first Get() invocation of the logger stream.
 **/
class Logfile {
 public:
  Logfile(string filename, string header) :
    filename(filename), header(header) {}

  ofstream& Log() {
    if (!headerWritten) {
      string filename = "/home/lvuser/" + filename;
      logfile.open(filename, ios::out);
      logfile << header << "\n";
    }
    return logfile;
  }

 private:
  string filename;
  string header;
  ofstream logfile;
  bool enabled = true;
  bool headerWritten = false;
};
