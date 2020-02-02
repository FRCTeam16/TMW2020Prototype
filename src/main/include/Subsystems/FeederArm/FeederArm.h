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

    void Run() override;

    void Instrument() override;

    void StartIntake(bool reverse = false){
        intakeEnabled = true;
        intakeReversed = reverse; 
    } 

    void StopIntake(){
        intakeEnabled = false;
    }

    void RunArm(double speed){
        armSpeed = speed;
    }

    void ZeroArmPosition() {
        armMotor->SetSelectedSensorPosition(0, 0, 50);
    }

    void ExtendClimberArms() {
        climberArms->Set(true);
    }

    void RetractClimberArms() {
        climberArms->Set(false);
    }

private : 
    std::shared_ptr<WPI_TalonSRX>  intakeMotor = RobotMap::gyroTalon; 
    std::shared_ptr<WPI_TalonFX> armMotor = RobotMap::armMotor; 
    std::shared_ptr<WPI_TalonFX> armMotorFollower = RobotMap::armMotorFollower;
    std::shared_ptr<Solenoid> climberArms = RobotMap::climberArms;

    bool intakeEnabled = false;
    bool intakeReversed = false;

    bool runArmControlled = false;
    double armSpeed = 0.0;

};
