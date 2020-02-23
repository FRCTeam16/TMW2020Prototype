#pragma once

#include <iostream>

#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>

#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

#include "RobotMap.h"
#include "Subsystems/Drive/DriveBase.h"
#include "OI.h"
#include "DMS/DmsProcessManager.h"
#include "DMS/StatusReporter.h"
#include "Subsystems/Vision/VisionSystem.h"
#include "Subsystems/Turret/Turret.h"
#include "Autonomous/World.h"
#include "Autonomous/AutoManager.h"
#include "Subsystems/FeederArm/FeederArm.h"
#include "Poses/ShotPoses.h"
#include "Subsystems/Color/ControlPanelSystem.h"



class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  // Subsystems
  static std::shared_ptr<DriveBase> driveBase;
  static std::unique_ptr<OI> oi;
  static std::shared_ptr<VisionSystem> visionSystem;
  static std::shared_ptr<Turret> turret;
  static std::shared_ptr<FeederArm> feederArm;
  static std::shared_ptr<ControlPanelSystem> controlPanelSystem;

private:
  void InitSubsystems();
  void RunSubsystems();
  void InstrumentSubsystems();
  void HandleGlobalInputs();

  void HandleArmBrakeButton();

  std::unique_ptr<RobotMap> robotMap;
  std::shared_ptr<StatusReporter> statusReporter;
  std::unique_ptr<DmsProcessManager> dmsProcessManager;
  std::unique_ptr<AutoManager> autoManager;
  
  std::shared_ptr<World> world;
  bool runInstrumentation = false;  // whether to run subsystem instrumentation

  // Operator input flags
  bool preloadPressed = false;    // prevent multiple ball queue commands
  std::unique_ptr<ShortShotPose> shortShotPose;
  std::unique_ptr<MediumShotPose> mediumShotPose;
  std::unique_ptr<TrenchShotPose> trenchShotPose;
  std::unique_ptr<LongShotPose> longShotPose;
  frc::DigitalInput toggleArmBreakModeButton{0};
  bool toggleArmBreakModeButtonPressed = false;
  
};
