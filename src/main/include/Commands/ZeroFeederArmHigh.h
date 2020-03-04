#pragma once



#include <frc/commands/Command.h>
#include "Robot.h"


class ZeroFeederArmHigh: public frc::Command {
public:
	ZeroFeederArmHigh();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
};
