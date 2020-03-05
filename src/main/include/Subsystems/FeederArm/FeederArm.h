#pragma once
#include "Subsystems/SubsystemManager.h"
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/LinearFilter.h>
#include "RobotMap.h"
#include "Util/BSPrefs.h"
#include "Util/PIDConfig.h"
#include <unordered_map>
#include <units/units.h>


struct ArmPIDConfig : PIDConfig {
    double velocity;
    double acceleration;
};

class FeederArm : public SubsystemManager {
public:
    enum Position { kZero, kDown, kShortShot, kPlayerStation, kVertical };
    explicit FeederArm();

    void Init() override;
    void InitTeleop();
    void InitAuto();
    void Run() override;
    void Instrument() override;

    void RunArm(double speed);
    void RunArmControlled();    // FIXME: Temp for testing
    void DebugSetPoint(double _setpoint);
    void SetArmPosition(Position position);
    void ZeroArmPosition(int position = 0);
    void SetArmBrakeMode(bool brakeEnabled);
    bool IsArmInPosition();

    void StartIntake(bool reverse = false);
    void StopIntake();
    void StartIntakeForColorSpin(double speed);
    void StopIntakeForColorSpin();
    bool IsIntakeJamDetected();

    void ExtendClimberArms();
    void RetractClimberArms();


private : 
    std::shared_ptr<WPI_VictorSPX>  intakeMotor = RobotMap::intakeMotor; 
    std::shared_ptr<WPI_TalonFX> armMotor = RobotMap::armMotor; 
    std::shared_ptr<WPI_TalonFX> armMotorFollower = RobotMap::armMotorFollower;
    std::shared_ptr<frc::Solenoid> climberRightArm = RobotMap::climberRightArm;
    std::shared_ptr<frc::Solenoid> climberLeftArm = RobotMap::climberLeftArm;
    std::shared_ptr<frc::PowerDistributionPanel> powerDistributionPanel = RobotMap::powerDistributionPanel;
    

    bool intakeEnabled = false;
    bool intakeReversed = false;
    bool intakeColorWheelMode = false;
    double colorModeSpeed = 0.5;

    bool climberExtended = false;
    bool climberMessageSent = false;

    bool runArmControlled = false;
    double armSpeed = 0.0;
    ArmPIDConfig armPIDConfig;
    double armSetpoint = 0.0;
    unordered_map<Position, double> armPositions;

    frc::LinearFilter<double> intakeSpikeDetector = frc::LinearFilter<double>::MovingAverage(5);
    bool intakeJamDetected = false;
    double jamThreshold = 50.0;
};
