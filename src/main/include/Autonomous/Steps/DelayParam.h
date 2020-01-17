#ifndef SRC_AUTONOMOUS_STEPS_DELAYPARAM_H_
#define SRC_AUTONOMOUS_STEPS_DELAYPARAM_H_

struct DelayParam {
	enum DelayType { kNone, kTime, kPosition };

	const DelayType delayType;
	const double value;

	DelayParam() : delayType(DelayType::kNone), value(0) {}
	DelayParam(DelayType _delayType, double _value) : delayType(_delayType), value(_value) {}
};



#endif /* SRC_AUTONOMOUS_STEPS_DELAYPARAM_H_ */
