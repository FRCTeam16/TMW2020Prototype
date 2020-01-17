#include "Util/DistanceControl.h"
#include "RobotMap.h"

DistanceControl::DistanceControl(std::shared_ptr<AnalogInput> _ultrasonic, double _target) :
	ultrasonic(_ultrasonic) {
	double P = 2.0;
	double I = 0.0;
	double D = 0.0;

	pid.reset(new PIDController(P, I, D, ultrasonic.get(), this));
	this->SetOutputRange(-0.3, 0.3);
	this->SetTarget(_target);
	pid->Enable();
}

DistanceControl::~DistanceControl() = default;

void DistanceControl::SetTarget(double _target) {
	pid->SetSetpoint(_target);
}

void DistanceControl::SetOutputRange(double _min, double _max) {
	pid->SetOutputRange(_min, _max);
}

void DistanceControl::PIDWrite(double _output) {
	output = _output;
}

void DistanceControl::SetInvert(bool _invert) {
	invert = (_invert) ? -1 : 1;
}

double DistanceControl::Get() const {
	return output * invert;
}

void DistanceControl::Enable() {
	pid->Enable();
}

void DistanceControl::Disable() {
	pid->Disable();
}
