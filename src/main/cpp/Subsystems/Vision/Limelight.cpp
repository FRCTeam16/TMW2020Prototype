/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Vision/Limelight.h"
#include "networktables/NetworkTableInstance.h"
#include <iostream>

Limelight::Limelight() {
    auto defaultInstance = nt::NetworkTableInstance::GetDefault();
    dataTable = defaultInstance.GetTable("limelight");
    SetStreamMode(StreamMode::USBMain);
}

SceneInfo Limelight::GetScene() const {
    SceneInfo si;

    si.hasTarget = dataTable->GetNumber("tv", 0) == 1;  // 0 or 1
    si.xOffset = dataTable->GetNumber("tx", 0.0);       // -27 - 27 degrees
    si.yOffset = dataTable->GetNumber("ty", 0.0);       // -20.5 - 20.5 degrees
    si.targetArea = dataTable->GetNumber("ta", 0.0);    // 0 - 100%
    si.skew = dataTable->GetNumber("ts", 0.0);          // -90 - 0 degrees
    si.latency = dataTable->GetNumber("tl", 0.0) + 11;  // pipeline latency + 11 for image capture
    return si;
}

int Limelight::GetCurrentPipeline() const {
    return static_cast<int>(dataTable->GetNumber("pipeline", -1));
}

void Limelight::SelectPipeline(const int pipelineNumber) {
    if (pipelineNumber >= 0 && pipelineNumber < 10) {
        dataTable->PutNumber("pipeline", pipelineNumber);
    } else {
        std::cerr << "Invalid pipeline # selected: " << pipelineNumber << std::endl;
    }
}

Limelight::CameraMode Limelight::GetCameraMode() const {
    return static_cast<CameraMode>(dataTable->GetNumber("camMode", -1));
}

void Limelight::SetCameraMode(CameraMode cameraMode) {
    // FIXME: Add enum for light states
    switch (cameraMode) {
        case CameraMode::DriverCamera:
            dataTable->PutNumber("ledMode", 1);
            break;
        default:
            dataTable->PutNumber("ledMode", 0);
            break;
    }
    const auto value = static_cast<int>(cameraMode);
    dataTable->PutNumber("camMode", value);
}

Limelight::CameraMode Limelight::ToggleCameraMode() {
    switch (GetCameraMode()) {
        case CameraMode::DriverCamera:
            SetCameraMode(CameraMode::ImageProcessing);
            return CameraMode::ImageProcessing;
        case CameraMode::ImageProcessing:
            SetCameraMode(CameraMode::DriverCamera);
            return CameraMode::DriverCamera;
        default:
            return CameraMode::Unknown;
    }
}

void Limelight::SetStreamMode(StreamMode streamMode) {
    const int nMode = static_cast<int>(streamMode);
    std::cout << "Limelight::SetStreamMode -> " << nMode << "\n";
    dataTable->PutNumber("stream", nMode);
}


double Limelight::CalculateDistance(double heightToCamera, double heightToTarget) const {
    const double angleToTarget = this->GetScene().yOffset;
    return (heightToTarget - heightToCamera) / tan(angleToTarget);
}