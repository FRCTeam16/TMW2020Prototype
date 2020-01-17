/*
 * CollisionDetector.cpp
 */

#include "Util/CollisionDetector.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

CollisionDetector::CollisionDetector(std::shared_ptr<BSGyro> imu_, double threshold_ ) :
	imu(imu_), threshold(threshold_) {
}

CollisionDetector::~CollisionDetector() = default;

const double UNIT_SCALE = 16384.0; //  = 1G  http://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html#a211525ea83d9728416661238a2a5402a


bool CollisionDetector::Detect(bool showOutput) {
	short xyz[3];
	imu->GetPigeon()->GetBiasedAccelerometer(xyz);


	double current_accel_x = xyz[0] / UNIT_SCALE;;
	double current_jerk_x = current_accel_x - last_accel_x;
	last_accel_x = current_accel_x;

	double current_accel_y = xyz[1] / UNIT_SCALE;
	double current_jerk_y = current_accel_y - last_accel_y;
	last_accel_y = current_accel_y;

	double absX = fabs(current_jerk_x);
	double absY = fabs(current_jerk_y);

	if (absX > largest_seen) {
		largest_seen = absX;
	}
	if (absY > largest_seen) {
		largest_seen = absY;
	}

	frc::SmartDashboard::PutNumber("Current Jerk X", current_jerk_x);
	frc::SmartDashboard::PutNumber("Current Jerk Y", current_jerk_y);
	if (showOutput) {
	std::cout <<  "Jerk: X " << current_jerk_x
			  << " | Y " << current_jerk_y
			  << "  | T" << threshold
			  << " | Largest Seen: " << largest_seen << "\n";
	}
	return (absX > threshold) || (absY > threshold);
}


