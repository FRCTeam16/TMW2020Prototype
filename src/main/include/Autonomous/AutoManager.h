#ifndef SRC_AUTONOMOUS_AUTOMANAGER_H_
#define SRC_AUTONOMOUS_AUTOMANAGER_H_

#include <frc/smartdashboard/SendableChooser.h>
#include "World.h"
#include "Strategy.h"
#include <map>

class AutoManager {
public:
	explicit AutoManager();
	~AutoManager() = default;
	void Init(std::shared_ptr<World> world);
	void Periodic(std::shared_ptr<World> world);
	void Instrument();

private:
	enum AutoStrategy {
		kNone = 0, kGoalSideSweepCenter, kGoalSideSweepOffset, kSnatchAndShoot, kDebug = 99
	};

	std::shared_ptr<frc::SendableChooser<int>> positions;
	std::shared_ptr<frc::SendableChooser<int>> strategies;
	std::unique_ptr<Strategy> CreateStrategy(const AutoStrategy &key, std::shared_ptr<World> word);
	std::unique_ptr<Strategy> currentStrategy;
	double startTime = -1;
	bool finalPhaseFired = false;

};

#endif /* SRC_AUTONOMOUS_AUTOMANAGER_H_ */
