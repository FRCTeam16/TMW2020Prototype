#include "Robot.h"
#include "OI.h"
#include "Util/BSPrefs.h" 
#include "Util/UtilityFunctions.h"
#include "Subsystems/Drive/TMW2019SwerveWheel.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/commands/Scheduler.h>

std::unique_ptr<OI> Robot::oi;
std::shared_ptr<DriveBase> Robot::driveBase;
std::shared_ptr<VisionSystem> Robot::visionSystem;
std::shared_ptr<Turret> Robot::turret;
std::shared_ptr<FeederArm> Robot::feederArm;
std::shared_ptr<ControlPanelSystem> Robot::controlPanelSystem;

void Robot::RobotInit() {
	std::cout << "Robot::RobotInit => \n";

	std::cout << "Reading properties\n";
	auto pref = BSPrefs::GetInstance();
	std::cout << "Sanity FLOFF: " << pref->GetDouble("FLOff", 0.0);
	std::cout << "Finished Reading properties\n";

	robotMap.reset(new RobotMap());
	oi.reset(new OI());
    driveBase.reset(new DriveBase());
	visionSystem.reset(new VisionSystem());
    statusReporter.reset(new StatusReporter());
	turret.reset(new Turret(visionSystem));
	feederArm.reset(new FeederArm());
    // statusReporter->Launch();
    dmsProcessManager.reset(new DmsProcessManager(statusReporter));

	autoManager.reset(new AutoManager());
	controlPanelSystem.reset(new ControlPanelSystem());

	RobotMap::gyro->ZeroYaw();

	shortShotPose.reset(new ShortShotPose());
	mediumShotPose.reset(new MediumShotPose());
	trenchShotPose.reset(new TrenchShotPose());
	longShotPose.reset(new LongShotPose());

	std::cout << "Robot::TeleopInit <=\n";
}

void Robot::HandleArmBrakeButton()
{
	const bool currentValue = toggleArmBreakModeButton.Get();		// current value unpressed is true

	if (!toggleArmBreakModeButtonPressed && !currentValue) {
		// Set to Coast mode
		toggleArmBreakModeButtonPressed = true;
		feederArm->SetArmBrakeMode(false);
	} else if (toggleArmBreakModeButtonPressed && currentValue) {
		// Button was pressed, but is no longer
		feederArm->SetArmBrakeMode(true);
		toggleArmBreakModeButtonPressed = false;
	} else if (toggleArmBreakModeButtonPressed && !currentValue) {
		// button is being held down
	}
}

void Robot::DisabledInit() {
	visionSystem->DisableVisionTracking();
}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	InstrumentSubsystems();
	HandleGlobalInputs();
	HandleArmBrakeButton();
	
}

void Robot::AutonomousInit() {
	cout << "AutonomousInit\n";
	RobotMap::gyro->ZeroYaw();
	world.reset(new World());
	autoManager->Init(world);
	InitSubsystems();
	feederArm->InitAuto();
	driveBase->InitAuto();
}
void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	autoManager->Periodic(world);
	RunSubsystems();
}

void Robot::TeleopInit() {
	InitSubsystems();
	driveBase->InitTeleop();
	visionSystem->ResetMaxOutputRange();
	feederArm->InitTeleop();
    std::cout << "Robot::TeleopInit <=\n";
}

void Robot::TeleopPeriodic() {
	// std::cout << "TeleopPeriodic\n";
    double startTime = frc::Timer::GetFPGATimestamp();
	frc::Scheduler::GetInstance()->Run();
	
	double threshold = 0.1;	// Used as general joystick deadband default
	// const bool lockWheels = oi->DL6->Pressed();  Removed for auto testing
	const bool lockWheels = false;

	const OI::DPad dPad = oi->GetGamepadDPad();
	const bool startButtonPressed = oi->GPStart->Pressed();
	frc::SmartDashboard::PutBoolean("Start Button Pressed", startButtonPressed);


	/**********************************************************
	 * Vision
	**********************************************************/
	if (oi->DR3->RisingEdge()) {
		turret->GetTurretRotation().ToggleVisionTracking();
	} 
	HandleGlobalInputs();

	
	/**********************************************************
	 * Turret Control
	**********************************************************/
	if (oi->GPA->RisingEdge()) {
		turret->ToggleShooterEnabled();
	}

	// Enable/Disable Soft Limits
	if (oi->DR11->RisingEdge()) {
		turret->GetTurretRotation().EnableTurretSoftLimits();
	} else if (oi->DR16->RisingEdge()) {
		turret->GetTurretRotation().DisableTurretSoftLimits();
	}
	
	// TODO: Fixme to track state
	const bool gamepadLTPressed = oi->GetGamepadLT() > 0.05;
	const bool gamepadRTPressed = oi->GetGamepadRT() > 0.05;
	double turretSpeed = 0.0;
	if (gamepadLTPressed) {
		// std::cout << "Turret => Turning Left\n";
		turretSpeed = oi->GetGamepadLT();
		turret->GetTurretRotation().SetOpenLoopTurretSpeed(turretSpeed);
	} else if (gamepadRTPressed) {
		// std::cout << "Turret => Turning Right\n";
		turretSpeed = oi->GetGamepadRT();
		turret->GetTurretRotation().SetOpenLoopTurretSpeed(-turretSpeed);
	} else {
		turret->GetTurretRotation().OpenLoopHaltTurret();
	}

	// Must be below open loop turret
	if (!startButtonPressed) {
		if (dPad == OI::DPad::kUp) {
			shortShotPose->Run();
		} else if (dPad == OI::DPad::kDown) {
			longShotPose->Run();
		} else if (dPad == OI::DPad::kLeft) {
			mediumShotPose->Run();
		} else if (dPad == OI::DPad::kRight) {
			trenchShotPose->Run();
		}
	}
	

	/**********************************************************
	 * FeederArm  Control
	 * Intake will be overridden by shooting
	**********************************************************/
	if (oi->DR1->Pressed()) {
		feederArm->StartIntake();
	} else if (oi->DR2->Pressed()) {
		feederArm->StartIntake(true);	// reversed
	} else if (fabs(oi->GetGamepadLeftStick()) > 0.25) {
		feederArm->StartIntakeForColorSpin();
	} else {
		feederArm-> StopIntake();
	}

	if (!preloadPressed && dPad == OI::DPad::kLeft) {
		turret->PreloadBall();
		preloadPressed = true;
	} else {
		preloadPressed = false;
	}


	if (oi->DL3->RisingEdge()){
		turret->PreloadBall();
	} else if (oi->DL1->Pressed()) {
		turret->StartFeeder();
	} else if (oi->DR5->Pressed()) {
		turret->StartFeeder(true);
	} else {
		turret->StopFeeder();
	}
	
	if (oi->GPB->RisingEdge()) {
		feederArm->SetArmPosition(FeederArm::Position::kDown);
	} 
	else if (oi->GPY->RisingEdge()) {
		feederArm->SetArmPosition(FeederArm::Position::kVertical);
	} 
	else if (oi->GPX->RisingEdge()) {
		feederArm->SetArmPosition(FeederArm::Position::kPlayerStation);
	} else if (oi->GPRB->RisingEdge()) {
		feederArm->RunArm(0.0);
		// feederArm->SetArmPosition(FeederArm::Position::kZero);	// TODO Remove preload bool, just a reminder of previous func
	} else if (oi->GPLB->RisingEdge()) {
		turret->PreloadBall();
	}
	/*else {
		double armSpeed = oi->GetGamepadRightStick();
		if (fabs(armSpeed)<0.05) {
			armSpeed = 0;
		}
		feederArm->RunArm(armSpeed);
	}
	*/

	if (oi->DL16->Pressed()) {
		turret->SetFeederAndShooterReversed(true);
	} else {
		turret->SetFeederAndShooterReversed(false);
	}

	/**********************************************************
	 * Climber Arms
	**********************************************************/
	
	if (startButtonPressed) {
		if (dPad == OI::DPad::kUp) {
			// shortShotPose->Run(false);		FIXME: temp
			feederArm->ExtendClimberArms();
		}
		if (dPad == OI::DPad::kDown) {
			feederArm->RetractClimberArms();
		}
	}


	/**********************************************************
	 * Testing and Diagnostics
	**********************************************************/
	const bool dmsMode = oi->DL11->Pressed();
	dmsProcessManager->SetRunning(dmsMode);

	
	/**********************************************************
	 * Drive Control
	**********************************************************/
	double twistInput = oi->GetJoystickTwist(threshold);
	double start = frc::Timer::GetFPGATimestamp();

	// Lock angle
	if (oi->DL2->Pressed()) {
		driveBase->SetTargetAngle(0.0);
		twistInput = driveBase->GetTwistControlOutput();
	}
	if (dmsMode) {
		// DriveBase input handled via DMS->Run()
	} else {
		if (!lockWheels) {
			double yMove = -oi->GetJoystickY(threshold);
			double xMove = oi->GetJoystickX();
			bool useGyro = true;
			if (oi->DR4->Pressed()) {
				// robot centric
				// xMove = std::copysign(xMove*xMove, xMove);
				// twistInput *= 0.5;
				useGyro = false;
			}
			driveBase->Crab(
				twistInput,
				yMove,
				xMove,
				useGyro);
		} else {
			driveBase->Crab(0, 0, 0, true);
		}
	}
	

	double now = frc::Timer::GetFPGATimestamp();
	double driveBaseTime = (now-start) * 1000;
	SmartDashboard::PutNumber("DriveBaseRun", driveBaseTime);
	RunSubsystems();
	InstrumentSubsystems();

	double elapsed = (frc::Timer::GetFPGATimestamp() - startTime) * 1000.0;
	SmartDashboard::PutNumber("Teleop Period (ms)", elapsed);
	SmartDashboard::PutNumber("Non-DriveBase Time (ms)", (elapsed - driveBaseTime));
}	


void Robot::InitSubsystems() {
    std::cout << "Robot::InitSubsystems =>\n";
	visionSystem->Init();
	turret->Init();
	feederArm->Init();
	controlPanelSystem->Init();
	// status & dms currently don't have init
	std::cout << "Robot::InitSubsystems <=\n";
}

void Robot::RunSubsystems() {
	// std::cout << "RunSubsystems() =>\n";
    double start = frc::Timer::GetFPGATimestamp();
    dmsProcessManager->Run();
	visionSystem->Run(); 
	turret->Run();
	feederArm->Run();
	controlPanelSystem->Run();
	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("Subsystem Times", (now-start) * 1000);
	// std::cout << "RunSubsystems() <=\n";
}

void Robot::InstrumentSubsystems() {
	autoManager->Instrument();
	frc::SmartDashboard::PutNumber("ArmPos", RobotMap::armMotor->GetSelectedSensorPosition());
	if (true || runInstrumentation) {
		RobotMap::gyro->Instrument();
		driveBase->Instrument();
		visionSystem->Instrument();
		turret->Instrument();
		feederArm->Instrument();
		controlPanelSystem->Instrument();
	}
}

void Robot::HandleGlobalInputs() {
	if (oi->DR9->RisingEdge()) {
		visionSystem->ToggleCameraMode();
	}
	if (oi->DR7->RisingEdge()) {
		visionSystem->GetLimelight()->SetStreamMode(Limelight::StreamMode::LimelightMain);
	} else if (oi->DR8->RisingEdge()) {
		visionSystem->GetLimelight()->SetStreamMode(Limelight::StreamMode::USBMain);
	} else if (oi->DR10->RisingEdge()) {
		visionSystem->GetLimelight()->SetStreamMode(Limelight::StreamMode::SideBySide);
	}

	// Only run instrumentation when button is pressed to avoid
	// network latency overhead
	runInstrumentation = oi->DL7->Pressed();
}


void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

