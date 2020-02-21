#pragma once

#include <frc/commands/Command.h>


class ZeroTurretPosition: public frc::Command {
public:
	ZeroTurretPosition();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
};
