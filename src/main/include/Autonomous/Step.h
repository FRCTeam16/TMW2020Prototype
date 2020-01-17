#pragma once

#include "World.h"
#include "Subsystems/Drive/CrabInfo.h"
#include <memory>


class Step {
public:
	Step() = default;
	virtual ~Step() = default;
	virtual bool Run(std::shared_ptr<World> world) = 0;
	virtual const CrabInfo* GetCrabInfo() { return crab.get(); }
	virtual bool IsManualDriveControl() const { return manualDriveControl; }
protected:
	std::unique_ptr<CrabInfo> crab { new CrabInfo() };
	bool manualDriveControl = false;
};

