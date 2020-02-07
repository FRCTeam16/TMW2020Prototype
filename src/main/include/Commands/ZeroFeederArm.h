#pragma once



#include <frc/commands/Command.h>
#include "Robot.h"


class ZeroFeederArm: public frc::Command {
public:
	ZeroFeederArm();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
};
