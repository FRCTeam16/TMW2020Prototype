#pragma once
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

class ColorSensor { 
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{i2cPort};

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  rev::ColorMatch m_colorMatcher;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  static constexpr frc::Color kBlueTarget = frc::Color(0.205, 0.461, 0.332);
  static constexpr frc::Color kGreenTarget = frc::Color(0.247, 0.494, 0.257);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.301, 0.522, 0.176);

 public:

 enum WheelColor{Blue, Yellow, Red, Green, Unknown};

  ColorSensor() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
  }

    WheelColor ReadColor(){
   
    frc::Color detectedColor = m_colorSensor.GetColor();

    /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    WheelColor wheelColor;

    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
      wheelColor = WheelColor::Blue; 
    } else if (matchedColor == kRedTarget) {
      colorString = "Red";
      wheelColor = WheelColor::Red;
    } else if (matchedColor == kGreenTarget) {
      colorString = "Green";
      wheelColor = WheelColor::Green;
    } else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
      wheelColor = WheelColor::Yellow;
    } else {
      colorString = "Unknown";
      wheelColor = WheelColor::Unknown;
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);

    return wheelColor
    
  }

};