#pragma once

#include <frc/commands/Command.h>
#include "Robot.h"


class RotateTurretToStart: public frc::Command {
public:
	RotateTurretToStart();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
};
