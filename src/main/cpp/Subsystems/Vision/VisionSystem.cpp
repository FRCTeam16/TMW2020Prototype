/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Vision/VisionSystem.h"

#include "Robot.h"
#include "Util/BSPrefs.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


VisionSystem::VisionSystem() {
    limelight.reset(new Limelight());

    double P = frc::SmartDashboard::PutNumber("Vision.x.P", 0.05);
    double I = frc::SmartDashboard::PutNumber("Vision.x.I", 0.0);
    double D = frc::SmartDashboard::PutNumber("Vision.x.D", 0.0);

    // double P = BSPrefs::GetInstance()->GetDouble("Vision.x.P", 0.5);
    // double I = BSPrefs::GetInstance()->GetDouble("Vision.x.I", 0.0);
    // double D = BSPrefs::GetInstance()->GetDouble("Vision.x.D", 0.0);
    /*double xThreshold =*/ BSPrefs::GetInstance()->GetDouble("Vision.x.threshold", 50.0);
    outputRange = BSPrefs::GetInstance()->GetDouble("Vision.x.range", 0.3);
    xoffPID.reset(new frc2::PIDController(P, I, D));
    xoffPID->SetSetpoint(0.0);
}

void VisionSystem::Run() {
    auto prefs = BSPrefs::GetInstance();
    // double P = prefs->GetDouble("Vision.x.P", 0.05);
    // double I = prefs->GetDouble("Vision.x.I", 0.0);
    // double D = prefs->GetDouble("Vision.x.D", 0.0);
    double P = frc::SmartDashboard::GetNumber("Vision.x.P", 0.05);
    double I = frc::SmartDashboard::GetNumber("Vision.x.I", 0.0);
    double D = frc::SmartDashboard::GetNumber("Vision.x.D", 0.00);

    double xThreshold = prefs->GetDouble("Vision.x.threshold", 50.0);
    xoffPID->SetPID(P, I, D);

    auto visionInfo = new VisionInfo();
    SceneInfo scene = limelight->GetScene();
    if (scene.hasTarget) {
        double xSpeed = std::clamp(xoffPID->Calculate(scene.xOffset + offsetDegrees), -outputRange, outputRange);
        visionInfo->hasTarget = true;
        visionInfo->xSpeed = -xSpeed;
        visionInfo->xOffset = scene.xOffset;
        visionInfo->inThreshold = fabs(scene.xOffset) <= xThreshold;
        visionInfo->targetArea = scene.targetArea;
    } else {
        xoffPID->Reset();
    }
    currentVisionInfo.reset(visionInfo);
}

std::shared_ptr<VisionInfo> VisionSystem::GetLastVisionInfo() {
    return currentVisionInfo;
}

void VisionSystem::ToggleCameraMode() {
    auto mode = limelight->ToggleCameraMode();
    std::cout << "VisionSystem::ToggleCameraMode - Toggled to mode: " << static_cast<int>(mode) << std::endl;
}

void VisionSystem::Instrument() {
    if (currentVisionInfo) {
        frc::SmartDashboard::PutBoolean("Vision Target?", currentVisionInfo->hasTarget);
        frc::SmartDashboard::PutNumber("Vision Threshold?", currentVisionInfo->inThreshold);
        frc::SmartDashboard::PutNumber("Vision xSpeed", currentVisionInfo->xSpeed);
    }
}

void VisionSystem::SetMaxOutputRange(double range) {
    std::cout << "VisionSystem::SetMaxOutputRange: " << range << "\n";
    outputRange = range;
}

void VisionSystem::ResetMaxOutputRange() {
    double range = BSPrefs::GetInstance()->GetDouble("Vision.x.range", 0.3);
    SetMaxOutputRange(range);
}

void VisionSystem::EnableVisionTracking() {
    limelight->SetCameraMode(Limelight::CameraMode::ImageProcessing);
}

void VisionSystem::DisableVisionTracking() {
    limelight->SetCameraMode(Limelight::CameraMode::DriverCamera);
}

bool VisionSystem::IsVisionTrackingEnabled() {
    return limelight->GetCameraMode() == Limelight::CameraMode::ImageProcessing;
}

void VisionSystem::SetOffsetDegrees(double offset)
{
    offsetDegrees = offset;
}