#ifndef SRC_SUBSYSTEMS_STATUSREPORTER_H_
#define SRC_SUBSYSTEMS_STATUSREPORTER_H_

#include "Subsystems/Drive/DriveInfo.h"
#include <frc/SerialPort.h>
#include <thread>


class StatusReporter {
public:
	StatusReporter();
	virtual ~StatusReporter() {}
	void Run();
	void Launch();

	void SetDmsMode(bool _mode) { dmsMode = _mode; }
	void SetDriveStatus(DriveInfo<int> _status) { driveStatus = _status; }
	void SetSteerStatus(DriveInfo<int> _status) { steerStatus = _status; }
private:
	bool running = false;
	std::thread reporterThread;
	std::unique_ptr<frc::SerialPort> serial;

	bool dmsMode = false;

	void SendData();

	DriveInfo<int> driveStatus;
	DriveInfo<int> steerStatus;
};

#endif /* SRC_SUBSYSTEMS_STATUSREPORTER_H_ */
