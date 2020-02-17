#include <iostream>
#include <vector>

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "RobotMap.h"
#include "Subsystems/Drive/DriveBase.h"
#include "Subsystems/Drive/CrabSpeed.h"
#include "Subsystems/Drive/DriveEncoderPIDSource.h"
#include "Subsystems/Drive/TMW2019SwerveWheel.h"
#include "Util/BSPrefs.h"


# define M_PI		3.14159265358979323846	/* pi */
//#define DEBUG 1

#ifdef DEBUG
#define D(x) x
#define DM(x) (std::cout << x << std::endl)
#else
#define D(x)
#define DM(x)
#endif


using namespace frc;

DriveBase::DriveBase() : Subsystem("DriveBase") {
	std::cout << "DriveBase::DriveBase =>\n";

	frontLeft.reset(new TMW2019SwerveWheel{"FL", RobotMap::driveBaseFrontLeftDrive, RobotMap::driveBaseFrontLeftSteer});
	frontRight.reset(new TMW2019SwerveWheel{"FR", RobotMap::driveBaseFrontRightDrive, RobotMap::driveBaseFrontRightSteer});
	rearLeft.reset(new TMW2019SwerveWheel{"RL", RobotMap::driveBaseRearLeftDrive, RobotMap::driveBaseRearLeftSteer});
	rearRight.reset(new TMW2019SwerveWheel{"RR",  RobotMap::driveBaseRearRightDrive, RobotMap::driveBaseRearRightSteer});
	wheels.push_back(frontLeft);
	wheels.push_back(frontRight);
	wheels.push_back(rearLeft);
	wheels.push_back(rearRight);
	std::cout << "  wheels initialized\n";

    crabSpeedTwist.reset(new CrabSpeed());

	InitializeOffsets();
	std::cout << "  offsets initialized\n";

    wheelbase.W = 20.5/2;
    wheelbase.X = 23.0;
    wheelbase.Y = 20.5/2;

    // Initialize wheels
	for (auto const& wheel : wheels) {
		wheel->InitializeSteering();
	}
	std::cout << "  steering initialized\n";
    InitializePIDs();
	std::cout << "DriveBase::DriveBase <=";
}

void DriveBase::InitializePIDs() {
	 // Initialize Drive Talons + PID feedback
	BSPrefs *prefs = BSPrefs::GetInstance();
	// const double izone = BSPrefs::GetInstance()->GetDouble("DriveControlTwistIZone", 0.0);

	if (driveControlTwist == nullptr) {
		driveControlTwist.reset(
					new PIDController(
						BSPrefs::GetInstance()->GetDouble("TwistP", 0.02),
						BSPrefs::GetInstance()->GetDouble("TwistI", 0.0),
						BSPrefs::GetInstance()->GetDouble("TwistD", 0.12),
						0.0, RobotMap::gyro.get(), crabSpeedTwist.get(), 0.02 ));
	} else {
		driveControlTwist->SetPID(
				BSPrefs::GetInstance()->GetDouble("TwistP", 0.02),
				BSPrefs::GetInstance()->GetDouble("TwistI", 0.0),
				BSPrefs::GetInstance()->GetDouble("TwistD", 0.12));
		// was setting izone in custom PID
	}
	driveControlTwist->SetContinuous(true);
	driveControlTwist->SetAbsoluteTolerance(2.0);
	driveControlTwist->Enable();
	driveControlTwist->SetOutputRange(-0.6, 0.6);
	driveControlTwist->SetInputRange(-180, 180);

	// Drive Motor PID	
	for (auto const& wheel : wheels) {
		wheel->InitializeDrivePID();
	}

	// Drive PID Control
	const double driveControlP = prefs->GetDouble("DriveControlP", 0.0);
	const double driveControlI = prefs->GetDouble("DriveControlI", 0.0);
	const double driveControlD = prefs->GetDouble("DriveControlD", 0.0);
	const double driveControlF = prefs->GetDouble("DriveControlF", 0.0);
	// const double driveControlIZone = prefs->GetDouble("DriveControlIZone", 0.0);
	if (driveControlEncoderSource == nullptr) {
		DriveInfo<std::shared_ptr<SwerveWheel>> motors;
		motors.FL = frontLeft;
		motors.FR = frontRight;
		motors.RL = rearLeft;
		motors.RR = rearRight;
		driveControlEncoderSource.reset(new FrontTwoAveragingDriveEncoderPIDSource(motors));
	}

	if (driveControlDistanceSpeed == nullptr) {
		driveControlDistanceSpeed.reset(new CrabSpeed());
	}

	if (driveControlSpeedController == nullptr) {
		driveControlSpeedController.reset(
				new PIDController(driveControlP, driveControlI, driveControlD, driveControlF,
						driveControlEncoderSource.get(),
						driveControlDistanceSpeed.get(),
						0.05));
//		driveControlSpeedController->SetIzone(driveControlIZone);
		driveControlSpeedController->Enable();
	} else {
		driveControlSpeedController->SetPID(driveControlP, driveControlI, driveControlD, driveControlF);
//		driveControlSpeedController->SetIzone(driveControlIZone);
	}
}

void DriveBase::InitDefaultCommand() {
}


// Put methods for controlling this subsystem
// here. Call these from Commands.

void DriveBase::InitializeOffsets() {
	positionOffsets.FL = BSPrefs::GetInstance()->GetDouble("FLOff", 0.0);
	positionOffsets.FR = BSPrefs::GetInstance()->GetDouble("FROff", 0.0);
	positionOffsets.RL = BSPrefs::GetInstance()->GetDouble("RLOff", 0.0);
	positionOffsets.RR = BSPrefs::GetInstance()->GetDouble("RROff", 0.0);
}

void DriveBase::Lock() {
	DriveInfo<double> steering;
	steering.FL = 0.375;
	steering.FR = 0.75;
	steering.RL = 0.75;
	steering.RR = 4.5;	// FIXME: Units need updating
	SetSteering(steering);

	DriveInfo<double> lockSpeed;
	SetDriveSpeed(lockSpeed);
}

DriveInfo<double> DriveBase::CalculatePositionOffsets() {
	positionOffsets.FL = frontLeft->GetSteerEncoderPositionInDegrees();
	positionOffsets.FR = frontRight->GetSteerEncoderPositionInDegrees();
	positionOffsets.RL = rearLeft->GetSteerEncoderPositionInDegrees();
	positionOffsets.RR = rearRight->GetSteerEncoderPositionInDegrees();
	return positionOffsets;
}


void DriveBase::ZeroTurnInfo() {
	turns.FL = 0;
	turns.FR = 0;
	turns.RL = 0;
	turns.RR = 0;
}

void DriveBase::ZeroDriveEncoders() {
	for (auto const& wheel : wheels) {
		wheel->ZeroDriveEncoder();
	}
}

void DriveBase::InitTeleop() {
	InitializePIDs();
	this->UseOpenLoopDrive();
	for (auto const& wheel : wheels) {
		wheel->InitTeleop();
	}
}

void DriveBase::InitAuto() {
	InitializePIDs();
	UseClosedLoopDrive();
	for (auto const& wheel : wheels) {
		wheel->InitAuto();
	}
}

void DriveBase::UseOpenLoopDrive() {
	std::cout << "*** UseOpenLoopDrive ***\n";
	for (auto const& wheel : wheels) {
		wheel->UseOpenLoopDrive();
	}
}


void DriveBase::UseClosedLoopDrive() {
	std::cout << "*** UseClosedLoopDrive ***\n";
	for (auto const& wheel : wheels) {
		wheel->UseClosedLoopDrive();
	}
}

void DriveBase::UseClosedLoopSpeedDrive() {
	std::cout << "*** UseClosedLoopSpeedDrive ***\n";
	for (auto const& wheel : wheels) {
		wheel->UseClosedLoopSpeedDrive();
	}
}

void DriveBase::Crab(double twist, double y, double x, bool useGyro) {
	// const double startTime = frc::Timer::GetFPGATimestamp();
	lastSpeedX = x;
	lastSpeedY = y;
	double FWD = y;
	double STR = x;

	if (useGyro) {
		assert(RobotMap::gyro.get() != nullptr);
		const double robotangle = RobotMap::gyro->GetYaw() * M_PI / 180;
		FWD =  y * cos(robotangle) + x * sin(robotangle);
		STR = -y * sin(robotangle) + x * cos(robotangle);
	}

	const double radius = sqrt(pow(2*wheelbase.Y, 2) + pow(wheelbase.X,2));
	double AP = STR - twist * wheelbase.X / radius;
	double BP = STR + twist * wheelbase.X / radius;
	double CP = FWD - twist * 2 * wheelbase.Y / radius;
	double DP = FWD + twist * 2 * wheelbase.Y / radius;

	// std::cout << "STEER(FWD = " << FWD << ", STR = " << STR << ", twist = " << twist << ")"
	// 	"| AP: " << AP <<
	// 	"| BP: " << BP <<
	// 	"| CP: " << CP <<
	// 	"| DP: " << DP << "\n";


	DriveInfo<double> setpoint(0.0);
	const double RadiansToDegrees = (180.0 / M_PI);

	if (DP != 0 || BP != 0) {
		setpoint.FL = atan2(BP,DP) * RadiansToDegrees;
	}
	if (BP != 0 || CP != 0) {
		setpoint.FR = atan2(BP, CP) * RadiansToDegrees;
	}
	if (AP != 0 || DP != 0) {
		setpoint.RL = atan2(AP, DP) * RadiansToDegrees;
	}
	if (AP != 0 || CP != 0) {
		setpoint.RR = atan2(AP, CP) * RadiansToDegrees;
	}

	SetSteering(setpoint);
	// const double steerEnd = (startTime - frc::Timer::GetFPGATimestamp()) * -1000.0;
	// SmartDashboard::PutNumber("Crab Steer (ms)", steerEnd);


	// const double driveStartTime = frc::Timer::GetFPGATimestamp();
	DriveInfo<double> speed(0.0);
	speed.FL = sqrt(pow(BP, 2) + pow(DP, 2));
	speed.FR = sqrt(pow(BP, 2) + pow(CP, 2));
	speed.RL = sqrt(pow(AP, 2) + pow(DP, 2));
	speed.RR = sqrt(pow(AP, 2) + pow(CP, 2));

	double speedarray[] = {fabs(speed.FL), fabs(speed.FR), fabs(speed.RL), fabs(speed.RR)};
	const double maxspeed = *std::max_element(speedarray, speedarray+4);

	DriveInfo<double> ratio;
	if (maxspeed > 1 || maxspeed < -1) {
		ratio.FL = speed.FL / maxspeed;
		ratio.FR = speed.FR / maxspeed;
		ratio.RL = speed.RL / maxspeed;
		ratio.RR = speed.RR / maxspeed;
	} else {
		ratio.FL = speed.FL;
		ratio.FR = speed.FR;
		ratio.RL = speed.RL;
		ratio.RR = speed.RR;
	}
	ratio.FR = -ratio.FR;
	ratio.RR = -ratio.RR;
	SetDriveSpeed(ratio);

	// const double driveEnd = (driveStartTime - frc::Timer::GetFPGATimestamp()) * -1000.0;
	// frc::SmartDashboard::PutNumber("Crab Drive (ms)", driveEnd);
	// const double end = (startTime - frc::Timer::GetFPGATimestamp()) * -1000.0;
	// frc::SmartDashboard::PutNumber("Crab Time (ms)", end);
}

void DriveBase::SetSteering(DriveInfo<double> setpoint) {
	if (driveFront) {
		DM("FL");
		frontLeft->SetSteerEncoderSetpoint(setpoint.FL, positionOffsets.FL, inv.FL);
		DM("FR");
		frontRight->SetSteerEncoderSetpoint(setpoint.FR, positionOffsets.FR, inv.FR);
		DM("RL");
		rearLeft->SetSteerEncoderSetpoint(setpoint.RL, positionOffsets.RL, inv.RL);
		DM("RR");
		rearRight->SetSteerEncoderSetpoint(setpoint.RR, positionOffsets.RR, inv.RR);
	} else {
		frontLeft->SetSteerEncoderSetpoint(setpoint.RR, positionOffsets.FL, inv.FL);
		frontRight->SetSteerEncoderSetpoint(setpoint.RL, positionOffsets.FR, inv.FR);
		rearLeft->SetSteerEncoderSetpoint(setpoint.FR, positionOffsets.RL, inv.RL);
		rearRight->SetSteerEncoderSetpoint(setpoint.FL, positionOffsets.RR, inv.RR);
	}
}

void DriveBase::SetDriveSpeed(DriveInfo<double> speed) {
	const float driveOutLimit = 50;
	const int hotCountLimit = 100;
	const int coolCountLimit = 1000;

	frontLeft->GetDriveOutputCurrent() > driveOutLimit ? hotCount.FL++ : hotCount.FL =0;
	frontRight->GetDriveOutputCurrent() > driveOutLimit ? hotCount.FR++ : hotCount.FR =0;
	rearLeft->GetDriveOutputCurrent() > driveOutLimit ? hotCount.RL++ : hotCount.RL =0;
	rearRight->GetDriveOutputCurrent() > driveOutLimit ? hotCount.RR++ : hotCount.RR =0;

	if (hotCount.FL == 0 && hotCount.FR == 0 && hotCount.RL == 0 && hotCount.RR == 0 ) {
		coolCount++;
	} else {
		coolCount = 0;
	}

	if (hotCount.FL > hotCountLimit && hotCount.FR > hotCountLimit &&
			hotCount.RL > hotCountLimit  && hotCount.RR > hotCountLimit) {
		driveLimit = 0.5;
	}

	if (coolCount > coolCountLimit) {
		driveLimit = 1.0;
	}

	// Check one of our drives to see if the mode is open or closed loop
	// FIXME: RPM Scale factor needs to be in preferences
	const bool isOpenLoop = frontLeft->IsOpenLoopDrive();	
	const float SCALE_FACTOR = (isOpenLoop) ? 1 : 6000;	// 13000 for Talon

	DriveInfo<double> speeds;
	
	if(driveFront) {
		speeds.FL = speed.FL * inv.FL * SCALE_FACTOR;
		speeds.FR = speed.FR * inv.FR * SCALE_FACTOR;
		speeds.RL = speed.RL * inv.RL * SCALE_FACTOR;
		speeds.RR = speed.RR * inv.RR * SCALE_FACTOR;
	} else {
		speeds.FL = speed.RR * inv.FL * SCALE_FACTOR;
		speeds.FR = speed.RL * inv.FR * SCALE_FACTOR;
		speeds.RL = speed.FR * inv.RL * SCALE_FACTOR;
		speeds.RR = speed.FL * inv.RR * SCALE_FACTOR;
	}

	// double start = frc::Timer::GetFPGATimestamp();
	if (isOpenLoop) {
		frontLeft->UseOpenLoopDrive(speeds.FL);
		frontRight->UseOpenLoopDrive(speeds.FR);
		rearLeft->UseOpenLoopDrive(speeds.RL);
		rearRight->UseOpenLoopDrive(speeds.RR);
	} else {
		frontLeft->UseClosedLoopSpeedDrive(speeds.FL);
		frontRight->UseClosedLoopSpeedDrive(speeds.FR);
		rearLeft->UseClosedLoopSpeedDrive(speeds.RL);
		rearRight->UseClosedLoopSpeedDrive(speeds.RR);
	}
	// double total = frc::Timer::GetFPGATimestamp() - start;
	// frc::SmartDashboard::PutNumber("DriveBaseSendSpeed", total);
}

void DriveBase::SetConstantVelocity(double twistInput, double velocity) {
	Crab(twistInput, velocity, 0, true);
}

void DriveBase::SetTargetAngle(double angle) {
	driveControlTwist->SetSetpoint(angle);
}

double DriveBase::GetTwistControlOutput() {
//	return driveControlTwist->Get();
	return crabSpeedTwist->Get();
}

double DriveBase::GetTwistControlError() {
	return driveControlTwist->GetError();
}

void DriveBase::SetTargetDriveDistance(double distance, double maxSpeed) {
	driveControlSpeedController->SetSetpoint(distance);
	driveControlSpeedController->SetOutputRange(-maxSpeed, maxSpeed);
	driveControlEncoderSource->SetInitialEncoderValue();
}

double DriveBase::GetDriveControlSetpoint() {
	return driveControlSpeedController->GetSetpoint();
}

double DriveBase::GetDriveControlEncoderPosition() {
	return driveControlEncoderSource->PIDGet();
}

double DriveBase::GetDriveControlOutput() {
	return driveControlDistanceSpeed->Get();
}

double DriveBase::GetDriveControlError() {
	return driveControlSpeedController->GetError();
}

double DriveBase::GetDriveControlP() {
	return driveControlSpeedController->GetP();
}

Wheelbase DriveBase::GetWheelbase() {
	return wheelbase;
}

double DriveBase::GetCrabTwistOutput() {
	return crabSpeedTwist->Get();
}

void DriveBase::Instrument() {
	bool detailed = false;
	if (detailed) {
		frc::SmartDashboard::PutNumber("FL V", frontLeft->GetDriveVelocity());
		frc::SmartDashboard::PutNumber("FR V", frontRight->GetDriveVelocity());
		frc::SmartDashboard::PutNumber("RL V", rearLeft->GetDriveVelocity());
		frc::SmartDashboard::PutNumber("RR V", rearRight->GetDriveVelocity());

		frc::SmartDashboard::PutNumber("FL A", frontLeft->GetDriveOutputCurrent());
		frc::SmartDashboard::PutNumber("FR A", frontRight->GetDriveOutputCurrent());
		frc::SmartDashboard::PutNumber("RL A", rearLeft->GetDriveOutputCurrent());
		frc::SmartDashboard::PutNumber("RR A", rearRight->GetDriveOutputCurrent());

		frc::SmartDashboard::PutNumber("FL Encoder", frontLeft->GetDriveEncoderPosition() );
		frc::SmartDashboard::PutNumber("FR Encoder", frontRight->GetDriveEncoderPosition() );
		frc::SmartDashboard::PutNumber("RL Encoder", rearLeft->GetDriveEncoderPosition() );
		frc::SmartDashboard::PutNumber("RR Encoder", rearRight->GetDriveEncoderPosition() );

		frc::SmartDashboard::PutNumber("FL Out Amps", frontLeft->GetDriveOutputCurrent() );
		frc::SmartDashboard::PutNumber("FR Out Amps", frontRight->GetDriveOutputCurrent() );
		frc::SmartDashboard::PutNumber("RL Out Amps", rearLeft->GetDriveOutputCurrent() );
		frc::SmartDashboard::PutNumber("RR Out Amps", rearRight->GetDriveOutputCurrent() );

		frc::SmartDashboard::PutNumber("FL Vel", frontLeft->GetDriveVelocity() );
		frc::SmartDashboard::PutNumber("FR Vel", frontRight->GetDriveVelocity() );
		frc::SmartDashboard::PutNumber("RL Vel", rearLeft->GetDriveVelocity() );
		frc::SmartDashboard::PutNumber("RR Vel", rearRight->GetDriveVelocity() );
	}
}

DriveInfo<double> DriveBase::GetDriveEncoderPositions() {
	DriveInfo<double> info;
	info.FL = frontLeft->GetDriveEncoderPosition();
	info.FR = frontRight->GetDriveEncoderPosition();
	info.RR = rearRight->GetDriveEncoderPosition();
	info.RL = rearLeft->GetDriveEncoderPosition();
	return info;
}

DriveInfo<double> DriveBase::GetDriveCurrent() {
	DriveInfo<double> info;
	info.FL = frontLeft->GetDriveOutputCurrent();
	info.FR = frontRight->GetDriveOutputCurrent();
	info.RR = rearRight->GetDriveOutputCurrent();
	info.RL = rearLeft->GetDriveOutputCurrent();
	return info;
}
 DriveInfo <int> DriveBase:: GetSteerEncoderPositions() {
 	 DriveInfo<int> info;
 	 info.FL = frontLeft->GetSteerEncoderPosition();
 	 info.FR = frontRight->GetSteerEncoderPosition();
 	 info.RR = rearRight->GetSteerEncoderPosition();
 	 info.RL = rearLeft->GetSteerEncoderPosition();
 	 return info;
 }

 DriveInfo <double> DriveBase::GetSteerCurrent() {
 	 DriveInfo<double> info;
 	 info.FL = frontLeft->GetSteerOutputCurrent();
 	 info.FR = frontRight->GetSteerOutputCurrent();
 	 info.RR = rearRight->GetSteerOutputCurrent();
 	 info.RL = rearLeft->GetSteerOutputCurrent();
 	 return info;
}

void DriveBase::DMSDrive(double speed) {
	for (auto const& wheel: wheels) {
		wheel->UseOpenLoopDrive(speed);
	}
}

void DriveBase::DMSSteer(double speed) {
	for(auto const& wheel : wheels) {
		wheel->UseOpenLoopSteer(speed);
	}
}

DriveInfo<int> DriveBase::GetDMSDriveVelocity() {
	DriveInfo<int> info;
	info.FL = frontLeft->GetDriveVelocity();
	info.FR = frontRight->GetDriveVelocity();
	info.RR = rearRight->GetDriveVelocity();
	info.RL = rearLeft->GetDriveVelocity();
	return info;
}

DriveInfo<int> DriveBase::GetDMSSteerVelocity() {
	DriveInfo<int> info;
	info.FL = frontLeft->GetSteerVelocity();
	info.FR = frontRight->GetSteerVelocity();
	info.RR = rearRight->GetSteerVelocity();
	info.RL = rearLeft->GetSteerVelocity();
	return info;
}

void DriveBase::Steer(float radian, float speed, float a) {
	std::cout << "Radian:" << radian << "|| Speed:" << speed << std::endl;

	A=a;

	thetaRC = M_PI - radian;  //convert steering angle to rear center wheel angle
	DriveInfo<double> steerRatio;

	if(thetaRC != M_PI / 2)	//If we are not driving straight forward...
	{
		if(thetaRC < M_PI / 2)	//Right Turn
		{
			RightTurn4Wheels();
		}
		else if(thetaRC > M_PI / 2)	//Left Turn
		{
			LeftTurn4Wheels();
		}
	}
	else	//thetaRC = M_PI / 2
	{
		theta.FL = M_PI / 2;
		theta.FR = M_PI / 2;
		theta.RL = M_PI / 2;
		theta.RR = M_PI / 2;

		steerRatio.FL = 1;
		steerRatio.FR = 1;
		steerRatio.RL = 1;
		steerRatio.RR = 1;
	}
	//Solve for fastest wheel speed
	double speedarray[] = {fabs(steerSpeed.FL), fabs(steerSpeed.FR), fabs(steerSpeed.RL), fabs(steerSpeed.RR)};

	 int length = 4;
     double maxspeed = speedarray[0];
     for(int i = 1; i < length; i++)
     {
          if(speedarray[i] > maxspeed)
                maxspeed = speedarray[i];
     }

	//Set ratios based on maximum wheel speed
	steerRatio.FL = steerSpeed.FL/maxspeed;
	steerRatio.FR = steerSpeed.FR/maxspeed;
	steerRatio.RL = steerSpeed.RL/maxspeed;
	steerRatio.RR = steerSpeed.RR/maxspeed;

	//Set drive speeds
	steerRatio.FL = -steerRatio.FL * speed;
	steerRatio.FR = steerRatio.FR * speed;
	steerRatio.RL = -steerRatio.RL * speed;
	steerRatio.RR = steerRatio.RR * speed;
	SetDriveSpeed(steerRatio);

	//Set Steering PID Setpoints
	DriveInfo<double> setPoint;
	setPoint.FL = (1.25 + 2.5/M_PI*theta.FL);
	setPoint.FR = (1.25 + 2.5/M_PI*theta.FR);
	setPoint.RL = (1.25 + 2.5/M_PI*theta.RL);
	setPoint.RR = (1.25 + 2.5/M_PI*theta.RR);

	SetSteering(setPoint);
}

void DriveBase::LeftTurn4Wheels()
{
	const double Z = ((A * wheelbase.X) * tan(M_PI - thetaRC));				//find turning radius

	//calculate angles based on turning radius
	theta.RL = M_PI - atan((Z - wheelbase.W) / (A * wheelbase.X));
	theta.RR = M_PI - atan((Z + wheelbase.W) / (A * wheelbase.X));
	theta.FR = M_PI / 2;
	theta.FL = M_PI / 2;

	if(A != 1) //not turning about front wheels
	{
		theta.FL = atan((Z - wheelbase.Y) / ((1 - A) * wheelbase.X));	//These are identical for right and left turns
		theta.FR = atan((Z + wheelbase.Y) / ((1 - A) * wheelbase.X));	//These are identical for right and left turns
	}
	//Solve each wheel turning radii (wheel speed)
	steerSpeed.FL = (Z - wheelbase.Y) / sin(theta.FL);
	steerSpeed.FR = (Z + wheelbase.Y) / sin(theta.FR);
	steerSpeed.RL = (Z - wheelbase.W) / sin(M_PI - theta.RL);
	steerSpeed.RR = (Z + wheelbase.W) / sin(M_PI - theta.RR);
}

void DriveBase::RightTurn4Wheels()
{
	const double Z = ((A * wheelbase.X) * tan(thetaRC));				//find turning radius

	//calculate angles based on turning radius
	theta.RL = atan((Z + wheelbase.W) / (A * wheelbase.X));
	theta.RR = atan((Z - wheelbase.W) / (A * wheelbase.X));
	theta.FR = M_PI / 2;
	theta.FL = M_PI / 2;


	if(A != 1)  //not turning about front wheels
	{
		theta.FR = M_PI - atan((Z - wheelbase.Y) / ((1 - A) * wheelbase.X));	//These are identical for right and left turns
		theta.FL = M_PI - atan((Z + wheelbase.Y) / ((1 - A) * wheelbase.X));	//These are identical for right and left turns
	}

	//Solve each wheel turning radii (wheel speed)
	steerSpeed.FL = (Z + wheelbase.Y) / sin(M_PI - theta.FL);
	steerSpeed.FR = (Z - wheelbase.Y) / sin(M_PI - theta.FR);
	steerSpeed.RL = (Z + wheelbase.W) / sin(theta.RL);
	steerSpeed.RR = (Z - wheelbase.W) / sin(theta.RR);
}


