/*
 * DriveInfo.h
 *
 *  Created on: Mar 27, 2017
 *      Author: User
 */

#ifndef SRC_UTIL_DRIVEINFO_H_
#define SRC_UTIL_DRIVEINFO_H_

template <typename T>
struct DriveInfo {
	T FL = 0;
	T FR = 0;
	T RL = 0;
	T RR = 0;

	DriveInfo() {}

	explicit DriveInfo(T value) : FL(value), FR(value), RL(value), RR(value) {}
	DriveInfo(T val1, T val2, T val3, T val4): FL(val1), FR(val2), RL(val3), RR(val4) {}
	friend DriveInfo<T> operator-(DriveInfo<T> lhs, const DriveInfo<T>& rhs) {
		return DriveInfo<T> {lhs.FL-rhs.FL, lhs.FR-rhs.FR, lhs.RL-rhs.RL, lhs.RR-rhs.RR};
	}
	friend DriveInfo<T> operator+(DriveInfo<T> lhs, const DriveInfo<T>& rhs) {
		return DriveInfo<T> {lhs.FL+rhs.FL, lhs.FR+rhs.FR, lhs.RL+rhs.RL, lhs.RR+rhs.RR};
	}
};




#endif /* SRC_UTIL_DRIVEINFO_H_ */
