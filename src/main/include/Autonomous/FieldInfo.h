#ifndef SRC_AUTONOMOUS_FIELDINFO_H_
#define SRC_AUTONOMOUS_FIELDINFO_H_

#include <string>

class FieldInfo {

public:
	FieldInfo(std::string gameData);
	FieldInfo();
	virtual ~FieldInfo();

	enum Location { Left, Right, Unknown};

	Location switchLocation;
	Location scaleLocation;
	Location farSwitchLocation;

private:
	void ParseGameData(std::string gameData);
};

#endif /* SRC_AUTONOMOUS_FIELDINFO_H_ */
