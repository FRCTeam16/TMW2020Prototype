#pragma once

struct PIDConfig {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0.000015;
    double kMaxOutput = 1.0;
    double kMinOutput = -1.0;

    double kRpm1 = 0.0;
};