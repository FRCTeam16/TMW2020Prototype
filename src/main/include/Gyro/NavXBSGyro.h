#pragma once

#include "Gyro/BSGyro.h"
#include "AHRS.h"
#include "Gyro/NavXCollisionDetector.h"

class NavXBSGyro : public BSGyro
{
public:
    NavXBSGyro(frc::I2C::Port i2c_port_id) : ahrs(new AHRS(i2c_port_id))
    {
    }
    
    double ReadRawYaw() override
    {
        // FIXME: offset here or in parent?
       return this->GetOffset() + ahrs->GetAngle();
    }

    void ZeroYaw() override
    {
        ahrs->ZeroYaw();
    }

    std::unique_ptr<CollisionDetector> GetCollisionDetector(double gThreshold) {
        auto detector = std::unique_ptr<NavXCollisionDetector>(new NavXCollisionDetector(ahrs, gThreshold));
        return std::move(detector);
    }



private:
    std::shared_ptr<AHRS> ahrs;
};