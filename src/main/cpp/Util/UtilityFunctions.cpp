#include "Util/UtilityFunctions.h"
#include <cmath>

double calculateLockAngle(double gyro_) {
    const int sign_multiplier = (gyro_ >= 0) ? 1 : -1;
    double answer = -1;
    double gyro = fabs(gyro_);

    if (gyro >= 0.0 && gyro < 15.0) {
        answer = 0.0;
    } else if (gyro  >= 15 && gyro < 60) {
        answer = 30.0;
    } else if (gyro >= 60 && gyro < 120) {
        answer = 90.0;
    } else if (gyro >= 120 && gyro < 165) {
        answer = 150.0;
    } else if (gyro >= 165 && gyro < 195) {
        answer = 180.0;
    // post-180 positive variables
    } else if (gyro >= 195 && gyro < 240) {
        answer = 210.0;
    } else if (gyro >= 240 && gyro < 285) {
        answer = 270.0;
    } else if (gyro >= 285 && gyro < 315) {
        answer = 300.0;
    } else if (gyro >= 315 && gyro <= 360) {
        answer = 360;
    }
    return sign_multiplier * answer;
   
}