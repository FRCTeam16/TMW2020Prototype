#include "OI.h"
#include "Util/BSButton.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "Commands/SetWheelOffsets.h"
#include "Commands/ZeroGyro.h"
#include "Commands/ZeroFeederArm.h"
#include "Commands/ZeroFeederArmHigh.h"
#include "Commands/ZeroTurretPosition.h"
#include "Commands/RotateTurretToStart.h"


# define M_PI		3.14159265358979323846	/* pi */



OI::OI() {
    // Process operator interface input here.
    gamepad.reset(new Joystick(2));

    driverRight.reset(new Joystick(1));

    driverLeft.reset(new Joystick(0));


    // SmartDashboard Buttons
    SmartDashboard::PutData("SetWheelOffsets2", new SetWheelOffsets());
    SmartDashboard::PutData("ZeroGyro2", new ZeroGyro());
    SmartDashboard::PutData("ZeroFeederArm2", new ZeroFeederArm());
    SmartDashboard::PutData("ZeroFeederArm HIGH", new ZeroFeederArmHigh());
    SmartDashboard::PutData("ZeroTurretPosition2", new ZeroTurretPosition());
    SmartDashboard::PutData("RotateTurretToStart", new RotateTurretToStart());

    GPX.reset(new BSButton(gamepad, 3));
    GPY.reset(new BSButton(gamepad, 4));
    GPB.reset(new BSButton(gamepad, 2));
    GPA.reset(new BSButton(gamepad, 1));
    GPLB.reset(new BSButton(gamepad, 5));
    GPRB.reset(new BSButton(gamepad, 6));
    GPBack.reset(new BSButton(gamepad, 7));
    GPStart.reset(new BSButton(gamepad, 8));
    DL1.reset(new BSButton(driverLeft, 1));
    DL2.reset(new BSButton(driverLeft, 2));
    DL3.reset(new BSButton(driverLeft, 3));
    DL4.reset(new BSButton(driverLeft, 4));
    DL5.reset(new BSButton(driverLeft, 5));
    DL6.reset(new BSButton(driverLeft, 6));
    DL7.reset(new BSButton(driverLeft, 7));
    DL8.reset(new BSButton(driverLeft, 8));
    DL9.reset(new BSButton(driverLeft, 9));
    DL10.reset(new BSButton(driverLeft, 10));
    DL11.reset(new BSButton(driverLeft, 11));
    DL12.reset(new BSButton(driverLeft, 12));
    DL13.reset(new BSButton(driverLeft, 13));
    DL14.reset(new BSButton(driverLeft, 14));
    DL16.reset(new BSButton(driverLeft, 16));

    DR1.reset(new BSButton(driverRight, 1));
    DR2.reset(new BSButton(driverRight, 2));
    DR3.reset(new BSButton(driverRight, 3));
    DR4.reset(new BSButton(driverRight, 4));
    DR5.reset(new BSButton(driverRight, 5));
    DR6.reset(new BSButton(driverRight, 6));
    DR7.reset(new BSButton(driverRight, 7));
    DR8.reset(new BSButton(driverRight, 8));
    DR9.reset(new BSButton(driverRight, 9));
    DR10.reset(new BSButton(driverRight, 10));
    DR11.reset(new BSButton(driverRight, 11));
    DR12.reset(new BSButton(driverRight, 12));
    DR16.reset(new BSButton(driverRight, 16));
}


std::shared_ptr<Joystick> OI::getDriverLeft() {
    return driverLeft;
}

std::shared_ptr<Joystick> OI::getDriverRight() {
    return driverRight;
}

std::shared_ptr<Joystick> OI::getGamepad() {
    return gamepad;
}

double OI::GetJoystickTwist(double threshold) {
    if (fabs(driverLeft->GetX()) < threshold) {
        return 0;
    } else {
        return driverLeft->GetX() / 2;
    }
}

double OI::GetJoystickX(double threshold) {
    if (fabs(driverRight->GetX()) < threshold) {
        return 0;
    } else {
        return driverRight->GetX();
    }
}

double OI::GetJoystickY(double threshold) {
    if (fabs(driverRight->GetY()) < threshold) {
        return 0;
    } else {
        return driverRight->GetY();
    }
}

double OI::getScaledJoystickRadians() {
    double steerAngle = M_PI / 2;
    steerAngle = driverRight->GetDirectionRadians();
    if (steerAngle > M_PI)
        steerAngle = M_PI;
    if (steerAngle < -M_PI)
        steerAngle = -M_PI;

    if (steerAngle < -M_PI / 2)
        steerAngle = -M_PI / 2 - steerAngle;
    else if (steerAngle < M_PI / 2)
        steerAngle = M_PI / 2 + steerAngle;
    else
        steerAngle = 3 * M_PI / 2 - steerAngle;
//	scalingFactor = driverJoystick->GetTwist()/2+1.5;

    return scaledRadians(steerAngle);
}

double OI::getLeftJoystickXRadians() {
    if(fabs(driverLeft->GetX())<.00)
        return M_PI/2;
    else
        return scaledRadians(M_PI/2 + driverLeft->GetX()*M_PI/2);
}

double OI::scaledRadians(double radians) {
    double scaledradians = M_PI / 2;
    double scalingFactor = 1.8;
    if (radians <= M_PI / 2)
        scaledradians =
                (-(M_PI / 2) / pow(pow(-M_PI / 2, 2), scalingFactor / 2))
                * pow(pow(radians - M_PI / 2, 2), scalingFactor / 2)
                + M_PI / 2;
    else
        //if(steerAngle <= M_PI)
        scaledradians = ((M_PI / 2) / pow((M_PI / 2), scalingFactor))
                        * pow(radians - M_PI / 2, scalingFactor) + M_PI / 2;

    return scaledradians;
}

double OI::GetGamepadLeftStick() {
    const double threshold = 0.1;
    if ((fabs(gamepad->GetRawAxis(1))) < threshold) {
        return 0;
    } else {
        return gamepad->GetRawAxis(1);
    }
}

double OI::GetGamepadRightStick() {
    const double threshold = 0.1;
    if ((fabs(gamepad->GetRawAxis(5))) < threshold) {
        return 0;
    } else {
        return gamepad->GetRawAxis(5);
    }
}

double OI::GetGamepadRT() {
    const double threshold = 0.1;
    if ((fabs(gamepad->GetRawAxis(3))) < threshold) {
        return 0;
    } else {
        return gamepad->GetRawAxis(3);
    }
}

double OI::GetGamepadLT() {
    const double threshold = 0.1;
    if ((fabs(gamepad->GetRawAxis(2))) < threshold) {
        return 0;
    } else {
        return gamepad->GetRawAxis(2);
    }
}

OI::DPad OI::TranslatePOV(int pov) {
    OI::DPad value = kUnknown;
    switch (pov) {
        case -1: value = kUnpressed;
            break;
        case 0: value = kUp;
            break;
        case 45: value = kUpRight;
            break;
        case 90: value = kRight;
            break;
        case 135: value = kDownRight;
            break;
        case 180: value = kDown;
            break;
        case 225: value = kDownLeft;
            break;
        case 270: value = kLeft;
            break;
        case 315: value = kUpLeft;
            break;
        default: value = kUnknown;
    }
    return value;
}

OI::DPad OI::GetGamepadDPad() {
    return TranslatePOV(gamepad->GetPOV());
}

OI::DPad OI::GetDRHat() {
    return TranslatePOV(driverRight->GetPOV());
}

void OI::SetGamepadLeftRumble(double rumble) {
    gamepad->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, rumble);
}

void OI::SetGamepadRightRumble(double rumble) {
    gamepad->SetRumble(frc::GenericHID::RumbleType::kRightRumble, rumble);
}

void OI::SetGamepadBothRumble(double rumble) {
    SetGamepadLeftRumble(rumble);
    SetGamepadRightRumble(rumble);
}
