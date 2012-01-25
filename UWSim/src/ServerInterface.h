/*
 * ServerInterface.h
 *
 *  Created on: 03/05/2010
 *      Author: mprats
 */

#ifndef SERVERINTERFACE_H_
#define SERVERINTERFACE_H_

#include "SimulatorConfig.h"
#include <OpenThreads/Thread>
#ifdef WIN32
#include <windows.h>
#endif

class ServerInterface: public OpenThreads::Thread {
protected:
	int port;	// server port
	int sockfd;	// socket file descriptor

public:
	ServerInterface(int port);

	virtual int processData(int socket)=0;

	/* Thread code */
	void run();

	~ServerInterface(){}
};


#endif /* SERVERINTERFACE_H_ */
