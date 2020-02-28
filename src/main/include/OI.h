#ifndef OI_H
#define OI_H

#include "frc/Joystick.h"
#include "Util/BSButton.h"

using namespace frc;

class OI {
private:
	std::shared_ptr<Joystick> gamepad;
	std::shared_ptr<Joystick> driverRight;
	std::shared_ptr<Joystick> driverLeft;
	double scaledRadians(double radians);
public:
	OI();
	std::shared_ptr<Joystick> getDriverLeft();
	std::shared_ptr<Joystick> getDriverRight();
	std::shared_ptr<Joystick> getGamepad();

	std::shared_ptr<BSButton> GPX;
	std::shared_ptr<BSButton> GPY;
	std::shared_ptr<BSButton> GPB;
	std::shared_ptr<BSButton> GPA;
	std::shared_ptr<BSButton> GPLB;
	std::shared_ptr<BSButton> GPRB;
	std::shared_ptr<BSButton> GPBack;
	std::shared_ptr<BSButton> GPStart;
	std::shared_ptr<BSButton> DL1;
	std::shared_ptr<BSButton> DL2;
	std::shared_ptr<BSButton> DL3;
	std::shared_ptr<BSButton> DL4;
	std::shared_ptr<BSButton> DL5;
	std::shared_ptr<BSButton> DL6;
	std::shared_ptr<BSButton> DL7;
	std::shared_ptr<BSButton> DL8;
	std::shared_ptr<BSButton> DL9;
	std::shared_ptr<BSButton> DL10;
	std::shared_ptr<BSButton> DL11;
	std::shared_ptr<BSButton> DL12;
	std::shared_ptr<BSButton> DL13;
	std::shared_ptr<BSButton> DL14;
	std::shared_ptr<BSButton> DL16;

	std::shared_ptr<BSButton> DR1;
	std::shared_ptr<BSButton> DR2;
	std::shared_ptr<BSButton> DR3;
	std::shared_ptr<BSButton> DR4;
	std::shared_ptr<BSButton> DR5;
	std::shared_ptr<BSButton> DR6;
	std::shared_ptr<BSButton> DR7;
	std::shared_ptr<BSButton> DR8;
	std::shared_ptr<BSButton> DR9;
	std::shared_ptr<BSButton> DR10;
	std::shared_ptr<BSButton> DR11;
	std::shared_ptr<BSButton> DR12;
	std::shared_ptr<BSButton> DR16;
	
	enum DPad {
		kUnpressed = -1,
		kUp = 0,
		kUpRight = 45,
		kRight = 90,
		kDownRight = 135,
		kDown = 180,
		kDownLeft = 225,
		kLeft = 270,
		kUpLeft = 315,
		kUnknown = 999
	};

	DPad GetGamepadDPad();
	DPad GetDRHat();
	DPad TranslatePOV(int povValue);

	double GetJoystickTwist(double threshold = 0.1);
	double GetJoystickX(double threshold = 0.1);
	double GetJoystickY(double threshold = 0.1);
	double GetGamepadLeftStick();
	double GetGamepadRightStick();
	double GetGamepadLT();
	double GetGamepadRT();
	void SetGamepadLeftRumble(double rumble);
	void SetGamepadRightRumble(double rumble);
	void SetGamepadBothRumble(double rumble);

	double getLeftJoystickXRadians();
	double getScaledJoystickRadians();
};

#endif
