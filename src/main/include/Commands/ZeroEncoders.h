#pragma once

#include <frc/commands/Command.h>
#include "Robot.h"


class ZeroEncoders: public frc::Command {
public:
	ZeroEncoders();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
};
