#include "Autonomous/Strategies/DebugAutoStrategy.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Delay.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/ClosedLoopDrive2.h"
#include "Autonomous/Steps/DriveToBump.h"
#include "Autonomous/Steps/Rotate.h"
#include "Autonomous/Steps/RotateUntilPast.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/SetGyroOffset.h"

#include "Autonomous/Steps/VisionControlledDrive.h"
#include "Autonomous/Steps/DriveToTarget.h"
#include "Autonomous/Steps/StopAtTarget.h"
#include "Autonomous/Steps/OpenDriveToDistance.h"
#include "Autonomous/Steps/SetVisionLight.h"
#include "Autonomous/Steps/StopAtTargetCount.h"
#include "Autonomous/Steps/SelectVisionPipeline.h"
#include "Autonomous/Steps/AlignToTarget.h"


DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	// DebugStraight();
	// DebugAutoHalt();
	// DebugControlled();
	std::cout << "--- DEBUG Autonomous Doing Nothing ---\n";
}
	

void DebugAutoStrategy::Init(std::shared_ptr<World> world) {
	std::cout << "DebugAutoStrategy::Init()\n";
	// AutoStartPosition startPosition = world->GetStartPosition();
	// const double angle = -180;
	// SetGyroOffset *step = new SetGyroOffset(angle);
	// step->Run(world);
}
