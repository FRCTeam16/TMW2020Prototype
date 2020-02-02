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
std::unique_ptr<Turret> Robot::turret;
std::unique_ptr<FeederArm> Robot::feederArm;


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
	RobotMap::gyro->ZeroYaw();

	std::cout << "Robot::TeleopInit <=\n";
}

void Robot::DisabledInit() {
	visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::DriverCamera);
	// initialized = false;
}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	InstrumentSubsystems();
	HandleGlobalInputs();
}

void Robot::AutonomousInit() {
	cout << "AutonomousInit\n";
	RobotMap::gyro->ZeroYaw();
	world.reset(new World());
	autoManager->Init(world);
	InitSubsystems();
	driveBase->InitTeleop();
	initialized = true;
}
void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	autoManager->Periodic(world);
}

void Robot::TeleopInit() {
    std::cout << "Robot::TeleopInit => initialized? " << initialized << "\n";
	if (!initialized) {
		InitSubsystems();
		driveBase->InitTeleop();
		initialized = true;
	} else {
		std::cout << " --- already initialized, ignoring\n";
	}
	visionSystem->ResetMaxOutputRange();
    std::cout << "Robot::TeleopInit <=\n";
}

void Robot::TeleopPeriodic() {
	// std::cout << "TeleopPeriodic\n";
    double startTime = frc::Timer::GetFPGATimestamp();
	frc::Scheduler::GetInstance()->Run();
	
	double threshold = 0.1;	// Used as general joystick deadband default
	// const bool lockWheels = oi->DL6->Pressed();  Removed for auto testing
	const bool lockWheels = false;

	const bool gamepadLTPressed = oi->GetGamepadLT() > 0.75;
	const bool gamepadRTPressed = oi->GetGamepadRT() > 0.75;


	/**********************************************************
	 * Vision
	**********************************************************/
	const bool visionMode = oi->DR3->Pressed();	// controls drive
	if (!visionMode) {
		visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::ImageProcessing);
	} else {
		visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::DriverCamera);
	}
	HandleGlobalInputs();

	
	/**********************************************************
	 * Turret Control
	**********************************************************/
	if (oi->GPA->RisingEdge()) {
		turret->ToggleShooterEnabled();
	}

	const double kTurretSpeed = frc::SmartDashboard::GetNumber("TurretSpeed", 1.0);;

	if (oi->GPX->Pressed()) {
		// std::cout << "Turret => Vision Tracking\n";
		turret->UseVisionTracking();
	} else {
		double turretSpeed = 0.0;
		if (gamepadLTPressed) {
			// std::cout << "Turret => Turning Left\n";
			turretSpeed = -kTurretSpeed;
		} else if (gamepadRTPressed) {
			// std::cout << "Turret => Turning Right\n";
			turretSpeed = kTurretSpeed;
		}
		turret->SetTurretSpeed(turretSpeed);
	}
	

	/**********************************************************
	 * FeederArm  Control
	**********************************************************/
	if (oi->DL1->Pressed()) {
		feederArm->StartIntake(); // TODO: handle reverse
	} else if (oi->DR2->Pressed()) {
		feederArm->StartIntake(true);
	} else {
		feederArm-> StopIntake();
	}

	if (oi->DR1->Pressed()) {
		feederArm->StartFeeder(); // TODO: handle reverse
	} else if (oi->DR5->Pressed()) {
		feederArm->StartFeeder(true);
	} else {
		feederArm-> StopFeeder();
	}

	double armSpeed = oi->GetGamepadRightStick();
	if (fabs(armSpeed)<0.05) {
		armSpeed = 0;
	}
	feederArm->RunArm(armSpeed);

	if (oi->DL5->RisingEdge()) {
		feederArm->ZeroArmPosition();
	}


	/**********************************************************
	 * Climber Arms
	**********************************************************/
	OI::DPad dPad = oi->GetGamepadDPad();
	if (dPad == OI::DPad::kUp) {
		feederArm->ExtendClimberArms();
	}
	if (dPad == OI::DPad::kDown) {
		feederArm->RetractClimberArms();
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
	if (dmsMode) {
		// DriveBase input handled via DMS->Run()
	} else {
		if (!lockWheels) {
			double yMove = -oi->GetJoystickY(threshold);
			double xMove = oi->GetJoystickX();
			bool useGyro = true;
			if (oi->DR4->Pressed()) {
				// robot centric
				xMove = std::copysign(xMove*xMove, xMove);
				twistInput *= 0.5;
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
	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("Subsystem Times", (now-start) * 1000);
	// std::cout << "RunSubsystems() <=\n";
}

void Robot::InstrumentSubsystems() {
	autoManager->Instrument();
	if (runInstrumentation) {
		frc::SmartDashboard::PutNumber("Penguin Temp", RobotMap::gyro->GetPigeon()->GetTemp());
		frc::SmartDashboard::PutNumber("RawYaw",RobotMap::gyro->ReadYaw());
		frc::SmartDashboard::PutNumber("Yaw",RobotMap::gyro->GetYaw());
		driveBase->Instrument();
		visionSystem->Instrument();
		turret->Instrument();
		feederArm->Instrument();
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

