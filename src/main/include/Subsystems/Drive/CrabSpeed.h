#pragma once

#include <frc/PIDOutput.h>

// TODO: PIDOutput has been deprecated, need to update usages
class CrabSpeed : public frc::PIDOutput
{
public:
	CrabSpeed();
	virtual ~CrabSpeed();

	void PIDWrite(double _output) override;
	double Get() const;
	
private:
	double output;
};


