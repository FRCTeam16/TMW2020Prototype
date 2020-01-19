#include "Robot.h"
#include "OI.h"
#include "Util/BSPrefs.h" 
#include "Util/UtilityFunctions.h"
#include "Subsystems/Drive/TMW2019SwerveWheel.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/commands/Scheduler.h>

std::unique_ptr<OI> Robot::oi;
std::shared_ptr<DriveBase> Robot::driveBase;
std::unique_ptr<VisionSystem> Robot::visionSystem;
std::unique_ptr<Turret> Robot::turret;
std::unique_ptr<Intake> Robot::intake;


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
	turret.reset(new Turret());
	intake.reset(new Intake());
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
	autoInitialized = false;			// flag for when autonomous routines are running
	InitSubsystems();
	driveBase->InitTeleop();
	initialized = true;
}
void Robot::AutonomousPeriodic() {
	// cout << "AutonomousPeriodic => TeleopPeriodic\n";
	// if teleop call removed add frc::Scheduler::GetInstance()->Run();
	TeleopPeriodic();
}

void Robot::TeleopInit() {
    std::cout << "Robot::TeleopInit => initialized? " << initialized << "\n";
	if (!initialized) {
		InitSubsystems();
		driveBase->InitTeleop();
		initialized = true;
		autoInitialized = false;

	// Debug
	frc::SmartDashboard::PutNumber("TurretSpeed", 0.2);

	} else {
		std::cout << " --- already initialized, ignoring\n";
	}
	visionSystem->ResetMaxOutputRange();
    std::cout << "Robot::TeleopInit <=\n";
}

void Robot::TeleopPeriodic() {
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
	if (!autoInitialized) {
		 if (visionMode) {
		 	visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::ImageProcessing);
		 } else {
		 	visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::DriverCamera);
		 }
	}
	HandleGlobalInputs();


	
	/**********************************************************
	 * Turret Control
	**********************************************************/
	if (oi->GPA->RisingEdge()) {
		turret->ToggleShooterEnabled();
	}

	const double kTurretSpeed = frc::SmartDashboard::GetNumber("TurretSpeed", 0.2);;
	double turretSpeed = 0.0;
	if (gamepadLTPressed) {
		turretSpeed = -kTurretSpeed;
	} else if (gamepadRTPressed) {
		turretSpeed = kTurretSpeed;
	}
	turret->SetTurretSpeed(turretSpeed);

	/**********************************************************
	 * Intake Control
	**********************************************************/
	if (oi->GPStart->RisingEdge()) {
		intake->ToggleEnabled();
	}

	/**********************************************************
	 * Testing and Diagnostics
	**********************************************************/
	const bool speedModeTest = false; // oi->DL7->Pressed();
	const bool dmsMode = oi->DL11->Pressed();
	dmsProcessManager->SetRunning(dmsMode);


	/**********************************************************
	 * Drive Control
	**********************************************************/
	double twistInput = oi->GetJoystickTwist(threshold);
	double start = frc::Timer::GetFPGATimestamp();
	if (speedModeTest) {
		// driveBase->SetConstantVelocity(twistInput, 0.60);
		// driveBase->Diagnostics();
	} else if (dmsMode) {
		// DriveBase input handled via DMS->Run()
	} else if (autoInitialized) {
		autoManager->Periodic(world);
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
	intake->Init();
	// status & dms currently don't have init
	std::cout << "Robot::InitSubsystems <=\n";
}

void Robot::RunSubsystems() {
    double start = frc::Timer::GetFPGATimestamp();
    dmsProcessManager->Run();
	visionSystem->Run(); 
	turret->Run();
	intake->Run();
	// liftController takes over driving so is in teleop loop
	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("Subsystem Times", (now-start) * 1000);
}

void Robot::InstrumentSubsystems() {
	autoManager->Instrument();
	if (runInstrumentation) {
		auto wheels = driveBase->GetWheels();
		frc::SmartDashboard::PutNumber("FL Encoder", wheels.FL->GetDriveEncoderPosition() );
		frc::SmartDashboard::PutNumber("FR Encoder", wheels.FR->GetDriveEncoderPosition() );
		frc::SmartDashboard::PutNumber("RL Encoder", wheels.RL->GetDriveEncoderPosition() );
		frc::SmartDashboard::PutNumber("RR Encoder", wheels.RR->GetDriveEncoderPosition() );

		frc::SmartDashboard::PutNumber("FL Out Amps", wheels.FL->GetDriveOutputCurrent() );
		frc::SmartDashboard::PutNumber("FR Out Amps", wheels.FR->GetDriveOutputCurrent() );
		frc::SmartDashboard::PutNumber("RL Out Amps", wheels.RL->GetDriveOutputCurrent() );
		frc::SmartDashboard::PutNumber("RR Out Amps", wheels.RR->GetDriveOutputCurrent() );

		frc::SmartDashboard::PutNumber("FL Vel", wheels.FL->GetDriveVelocity() );
		frc::SmartDashboard::PutNumber("FR Vel", wheels.FR->GetDriveVelocity() );
		frc::SmartDashboard::PutNumber("RL Vel", wheels.RL->GetDriveVelocity() );
		frc::SmartDashboard::PutNumber("RR Vel", wheels.RR->GetDriveVelocity() );

		// see DriveBase::Instrment for smartdashboard yaw 
		frc::SmartDashboard::PutNumber("Penguin Temp", RobotMap::gyro->GetPigeon()->GetTemp());

		driveBase->Instrument();
		visionSystem->Instrument();
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

