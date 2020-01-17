#pragma once



#include <frc/commands/Command.h>
#include "Robot.h"


class ZeroGyro: public frc::Command {
public:
	ZeroGyro();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
};
