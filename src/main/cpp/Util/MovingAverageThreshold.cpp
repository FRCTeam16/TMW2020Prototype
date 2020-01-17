/*
 * MovingAverageThreshold.cpp
 *
 *  Created on: Apr 13, 2017
 *      Author: User
 */

#include "Util/MovingAverageThreshold.h"
#include <iostream>

MovingAverageThreshold::~MovingAverageThreshold() = default;

void MovingAverageThreshold::Configure(double _threshold, int _windowSize) {
	threshold = _threshold;
	window = _windowSize;
	if (values.size() > window) {
		for (unsigned int i=0; i < values.size() - window; i++) {
			values.pop_front();
		}
	}

}

void MovingAverageThreshold::AddValue(double value) {
//	std::cout << "Add Value: " << value << "\n";
	if (values.size() == window) {
		values.pop_front();
	}
	values.push_back(value);
}

bool MovingAverageThreshold::Check() {
//	std::cout << "Check: values size: " << values.size() << "\n";
	if (values.size() != window) {
		return false;
	}

	double sum = 0.0;
	for (double value : values) {
		sum += value;
	}
	lastAverage = sum / (double) values.size();
	std::cout << "Sum = " << sum << " Average: " << lastAverage << "\n";
	return lastAverage >= threshold;
}

void MovingAverageThreshold::Reset() {
	values.clear();
	lastAverage = 0.0;
}

double MovingAverageThreshold::GetLastAverage() {
	return lastAverage;
}

