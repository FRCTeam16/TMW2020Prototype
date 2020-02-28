#include "Subsystems/Color/ColorSensor.h"



/**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  static constexpr frc::Color kBlueTarget = frc::Color(0.1390, 0.4237, 0.4371);
  static constexpr frc::Color kGreenTarget = frc::Color(0.1744, 0.5677, 0.2579);
  static constexpr frc::Color kRedTarget = frc::Color(0.4908, 0.3577, 0.1512);
  static constexpr frc::Color kYellowTarget = frc::Color(0.3123, 0.5543, 0.1334);


ColorSensor::ColorSensor(double minConfidence) : minConfidence(minConfidence)
  {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
  }

WheelColor ColorSensor::ReadColor()
{
    frc::Color detectedColor = m_colorSensor.GetColor();

    /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    WheelColor wheelColor;

    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (confidence >= minConfidence)
    {
      if (matchedColor == kBlueTarget)
      {
        colorString = "Blue";
        wheelColor = WheelColor::Blue;
      }
      else if (matchedColor == kRedTarget)
      {
        colorString = "Red";
        wheelColor = WheelColor::Red;
      }
      else if (matchedColor == kGreenTarget)
      {
        colorString = "Green";
        wheelColor = WheelColor::Green;
      }
      else if (matchedColor == kYellowTarget)
      {
        colorString = "Yellow";
        wheelColor = WheelColor::Yellow;
      }
      else
      {
        colorString = "Unknown";
        wheelColor = WheelColor::Unknown;
      }
    }
    else
    {
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

    return wheelColor;
}