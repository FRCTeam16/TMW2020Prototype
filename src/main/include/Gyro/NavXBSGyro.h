#pragma once

#include "Gyro/BSGyro.h"
#include "AHRS.h"

class NavXBSGyro : public BSGyro
{
public:
    NavXBSGyro(frc::I2C::Port i2c_port_id) : ahrs(new AHRS(i2c_port_id))
    {
    }
    
    double ReadRawYaw() override
    {
        return ahrs->GetAngle();
    }

    void ZeroYaw() override
    {
        ahrs->ZeroYaw();
    }


private:
    std::shared_ptr<AHRS> ahrs;
}