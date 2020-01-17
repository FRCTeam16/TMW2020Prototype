#include "Util/RampUtil.h"
#include <iostream>
#include <cmath>

double RampUtil::RampUp(double value, double elapsedTime, double rampTime, double minSpeed) {
	 /*
		 /  |
	   /    |
	  -------
	 */
	double speed = value;
	if (elapsedTime < rampTime) {
		speed = (value / rampTime) * elapsedTime;
		std::cout << "RampUp: " << elapsedTime << " Profiled Speed: " << speed << "\n";
		if (fabs(speed) < minSpeed) {
			speed = std::copysign(minSpeed, value);
		}
	} else {
		speed = value;
	}
	return speed;
}

double RampUtil::RampDown(double baseSpeed, double currentPosition, double target, double threshold, double minSpeed) {
	double speed = baseSpeed;
	double error = target - currentPosition;
	if (fabs(error) < fabs(threshold) ) {
		speed = baseSpeed * (error / threshold);
		std::cout << "RampDown Error: " << error << " Profiled Speed: " << speed << "\n";
		if (fabs(speed) < minSpeed) {
			speed = std::copysign(minSpeed, baseSpeed);
			std::cout << "RampDown - keeping minimum speed of " << speed;
		}
	}
	return speed;
}
