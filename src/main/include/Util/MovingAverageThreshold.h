/*
 * MovingAverageThreshold.h
 *
 *  Created on: Apr 13, 2017
 *      Author: User
 */

#ifndef SRC_UTIL_MOVINGAVERAGETHRESHOLD_H_
#define SRC_UTIL_MOVINGAVERAGETHRESHOLD_H_

#include <deque>

class MovingAverageThreshold {
private:
	double threshold;
	unsigned int window;
	std::deque<double> values;
	double lastAverage = 0.0;
public:
	MovingAverageThreshold(double _threshold, unsigned int _windowSize) : threshold(_threshold), window(_windowSize) {}
	virtual ~MovingAverageThreshold();
	void AddValue(double value);
	void Configure(double _threshold, int _windowSize);
	bool Check(); 	// True if flagged, False otherwise
	void Reset();
	double GetLastAverage();
};

#endif /* SRC_UTIL_MOVINGAVERAGETHRESHOLD_H_ */
