#include <iostream>
#include "Autonomous/Steps/ConcurrentStep.h"

ConcurrentStep::ConcurrentStep(std::initializer_list<Step*> stepList, bool _haltOnEnd)
	: haltOnEnd(_haltOnEnd)
{
	for (Step* step : stepList) {
		steps.push_back(new WrappedStep(step));
	}
	std::cout << "Loaded " << steps.size() << " steps to process concurrently\n";
}

ConcurrentStep::~ConcurrentStep() {
	for (auto it = steps.begin(); it != steps.end(); ++it) {
		delete *it;
	}
	steps.clear();
}

bool ConcurrentStep::Run(std::shared_ptr<World> world) {
	bool finished = true;
	int counter = 0;
	for (auto step : steps) {
		if (debug) std::cout << "==> ConcurrentStep::Run(): " << counter++ << "\n";
		finished &= step->Run(world);
	}
	if (debug) std::cout << "Concurrent::Run complete, finished? " << finished << "\n";
	return finished;
}


const CrabInfo* ConcurrentStep::GetCrabInfo() {
	WrappedStep *step = steps.front();
	const bool finished = step->IsFinished();
	if (haltOnEnd && finished) {
		if (debug) std::cout << "ConcurrentStep Drive Step has finished, returning stop.  HaltOnEnd? " << haltOnEnd << "\n";
		return STOP.get();
	} else {
		if (debug) std::cout << "ConcurrentStep returning real step info [finished? " << finished << "]\n";
		return step->GetStep()->GetCrabInfo();
	}
}

/*****************************************************************************/

WrappedStep::~WrappedStep() {
	delete step;
}

bool WrappedStep::Run(std::shared_ptr<World> world) {
	if (!finished) {
		finished = step->Run(world);
	}
	return finished;
}

bool WrappedStep::IsFinished() {
	return finished;
}
