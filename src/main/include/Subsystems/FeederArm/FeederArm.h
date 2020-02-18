#pragma once
#include "Subsystems/SubsystemManager.h"
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include "RobotMap.h"
#include "Util/BSPrefs.h"
#include "Util/PIDConfig.h"


class FeederArm : public SubsystemManager {
public:
    explicit FeederArm();

    void Init() override;
    void InitTeleop();
    void InitAuto();
    void Run() override;
    void Instrument() override;

    void RunArm(double speed);
    void RunArmControlled();    // FIXME: Temp for testing
    void DebugSetPoint(double _setpoint);
    void ZeroArmPosition();
    void SetArmBrakeMode(bool brakeEnabled);

    void StartIntake(bool reverse = false);
    void StopIntake();

    void ExtendClimberArms();
    void RetractClimberArms();


private : 
    std::shared_ptr<WPI_TalonSRX>  intakeMotor = RobotMap::intakeMotor; 
    std::shared_ptr<WPI_TalonFX> armMotor = RobotMap::armMotor; 
    std::shared_ptr<WPI_TalonFX> armMotorFollower = RobotMap::armMotorFollower;
    std::shared_ptr<frc::Solenoid> climberArms = RobotMap::climberArms;

    bool intakeEnabled = false;
    bool intakeReversed = false;

    bool climberExtended = false;
    bool climberMessageSent = false;

    bool runArmControlled = false;
    double armSpeed = 0.0;
    PIDConfig armPIDConfig;
    double armSetpoint = 0.0;

};
