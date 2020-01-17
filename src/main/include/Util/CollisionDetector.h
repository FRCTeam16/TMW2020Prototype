/*
 * CollisionDetector.h
 */

#ifndef SRC_UTIL_COLLISIONDETECTOR_H_
#define SRC_UTIL_COLLISIONDETECTOR_H_

#include "BSGyro.h"

class CollisionDetector {
public:
	explicit CollisionDetector(std::shared_ptr<BSGyro> imu, double threshold_=1.0);
	virtual ~CollisionDetector();
	bool Detect(bool showOutput = false);
private:
	std::shared_ptr<BSGyro> imu;
	const double threshold;
	double last_accel_x = 0.0;
	double last_accel_y = 0.0;
	double largest_seen = 0.0;
};

#endif /* SRC_UTIL_COLLISIONDETECTOR_H_ */
