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
    explicit FeederArm() {
        armMotorFollower->Follow(*armMotor);
        armMotorFollower->SetInverted(true);
    }

    void Init() override {
        frc::SmartDashboard::PutNumber("IntakeSpeed", 1.0);
        frc::SmartDashboard::PutNumber("FeederSpeed", -0.8);

    }

    void Run() override {
        double intakeSpeed = 0.0;
        if (intakeEnabled) {
            intakeSpeed = frc::SmartDashboard::GetNumber("IntakeSpeed", 1.0);
            if (intakeReversed) {
                intakeSpeed = -intakeSpeed;
            }
        }
        intakeMotor->Set(intakeSpeed);

        double feederSpeed = 0.0;
        if (feederEnabled) {
            feederSpeed = frc::SmartDashboard::GetNumber("FeederSpeed", -0.8);
            if (feederReversed) {
                feederSpeed = -feederSpeed;
            }
        }
        feederMotor->Set(feederSpeed);

        if (!runArmControlled) {
            armMotor->Set(ControlMode::PercentOutput, armSpeed);
        } else {
            // run magic motion
            // see https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B/MotionMagic/src/main/cpp/Robot.cpp
        }
        frc::SmartDashboard::PutNumber("ArmPos", armMotor->GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber("ArmPercent", armSpeed);
    }

    void Instrument() override {
        
    }

    void StartIntake(bool reverse = false){
        intakeEnabled = true;
        intakeReversed = reverse; 
    } 

    void StopIntake(){
        intakeEnabled = false;
    }

    void StartFeeder(bool reverse = false){
        feederEnabled = true;
        feederReversed = reverse;
    }

    void StopFeeder(){
        feederEnabled = false;
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
    std::shared_ptr<rev::CANSparkMax> feederMotor = RobotMap::feederMotor;
    std::shared_ptr<Solenoid> climberArms = RobotMap::climberArms;

    bool intakeEnabled = false;
    bool intakeReversed = false;
    bool feederEnabled = false;
    bool feederReversed = false;

    bool runArmControlled = false;
    double armSpeed = 0.0;

};
