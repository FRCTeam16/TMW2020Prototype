/*
 * SubsystemManager.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_SUBSYSTEMS_SUBSYSTEMMANAGER_H_
#define SRC_SUBSYSTEMS_SUBSYSTEMMANAGER_H_

class SubsystemManager {
public:

	virtual void Init() {}
	virtual void Run() {}
	virtual void Instrument() {}
};



#endif /* SRC_SUBSYSTEMS_SUBSYSTEMMANAGER_H_ */
