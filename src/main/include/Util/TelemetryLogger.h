#pragma once

#include <fstream>
#include <thread>

class TelemetryLogger {
public:
	TelemetryLogger();
	virtual ~TelemetryLogger();
	void Run();
	void Begin();
	void End();
	void Launch();
private:
	std::thread telemetryThread;
	bool running = false;
	std::ofstream logFile;
	void Log();
};
