#pragma once

#include "Gyro/BSGyro.h"
#include "Gyro/PigeonCollisionDetector.h"
#include <ctre/Phoenix.h>

class PigeonBSGyro : public BSGyro
{
public:
    PigeonBSGyro(WPI_TalonSRX *talon) : pigeon(new PigeonIMU(talon))
    {
        std::cout << "Constructed BSGyro attached to TalonSRX\n";
    }

    PigeonBSGyro(int canID) : pigeon(new PigeonIMU(canID))
    {
        std::cout << "Constructed BSGyro with device ID " << canID << "\n";
    }

    ~PigeonBSGyro() {}

    double ReadRawYaw() override
    {
        double ypr[3];
        int errorCode = pigeon->GetYawPitchRoll(ypr);
        if (ErrorCode::OK != static_cast<ErrorCode>(errorCode))
        {
            std::cerr << "Error from gyro: " << errorCode << "\n";
        }

        double trimmedYaw = fmod(ypr[0], 360.0);
        // SmartDashboard::PutNumber("PigeonYaw", ypr[0]);
        return GetOffset() + trimmedYaw;
    }

    void ZeroYaw() override
    {
        pigeon->SetYaw(0, 0);
    }

    std::unique_ptr<CollisionDetector> GetCollisionDetector(double gThreshold) override
    {
        auto detector = std::unique_ptr<PigeonCollisionDetector>(new PigeonCollisionDetector(pigeon, gThreshold));
        return std::move(detector);
    }

    void GyroSpecificInstrument() override
    {
        frc::SmartDashboard::PutNumber("Penguin Temp", pigeon->GetTemp());
    }

private:
    std::shared_ptr<PigeonIMU> pigeon;
};
