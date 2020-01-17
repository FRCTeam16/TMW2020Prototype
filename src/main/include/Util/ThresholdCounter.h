#ifndef SRC_UTIL_THRESHOLDCOUNTER_H_
#define SRC_UTIL_THRESHOLDCOUNTER_H_

class ThresholdCounter {
public:
	ThresholdCounter(double threshold, int numberOfChecks);
	virtual ~ThresholdCounter();
	bool Check(double input);
	void Reset();

private:
	const double threshold;
	const int numberOfChecks;
	int checkCount = 0;
};

#endif /* SRC_UTIL_THRESHOLDCOUNTER_H_ */
