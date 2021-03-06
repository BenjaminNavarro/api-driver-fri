//  ---------------------- Doxygen info ----------------------
//! \file FastResearchInterface.cpp
//!
//! \brief
//! Implementation file for the class FastResearchInterface
//!
//! \details
//! The class FastResearchInterface provides a basic low-level interface
//! to the KUKA Light-Weight Robot IV For details, please refer to the file
//! FastResearchInterface.h.
//!
//! \date December 2014
//!
//! \version 1.2
//!
//!	\author Torsten Kroeger, tkr@stanford.edu\n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//! \n
//! \n
//! \copyright Copyright 2014 Stanford University\n
//! \n
//! Licensed under the Apache License, Version 2.0 (the "License");\n
//! you may not use this file except in compliance with the License.\n
//! You may obtain a copy of the License at\n
//! \n
//! http://www.apache.org/licenses/LICENSE-2.0\n
//! \n
//! Unless required by applicable law or agreed to in writing, software\n
//! distributed under the License is distributed on an "AS IS" BASIS,\n
//! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n
//! See the License for the specific language governing permissions and\n
//! limitations under the License.\n
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <FastResearchInterface.h>
#include <Console.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <stdarg.h>
#include <OSAbstraction.h>

#include <iostream>
using namespace std;

#define MAX_ROBOT_NAME_LENGTH               256
#define MAX_OUTPUT_PATH_LENGTH              256
#define MAX_FILE_NAME_LENGTH                256
#define SIZE_OF_ROBOT_STATE_STRING          4096


// ****************************************************************
// Constructor
//
FastResearchInterface::FastResearchInterface(float cycle_time, int server_port)
{
	int FuntionResult                       =   0;

	struct sched_param SchedulingParamsKRCCommunicationThread
	,   SchedulingParamsMainThread;

	pthread_attr_t AttributesKRCCommunicationThread;

	this->PriorityKRCCommunicationThread    = 98;
	this->PriorityMainThread                = 50;
	this->PriorityOutputConsoleThread       = 5;

	this->CycleTime                         = cycle_time;
	this->ServerPort                        = server_port;

	this->OutputConsole                     =   new Console(PriorityOutputConsoleThread);

	this->KRCCommunicationThreadIsRunning   =   true;
	this->NewDataFromKRCReceived            =   false;
	this->ThreadCreated                     =   false;

	this->CurrentControlScheme              =   FastResearchInterface::JOINT_POSITION_CONTROL;

	this->RobotStateString      =   new char[SIZE_OF_ROBOT_STATE_STRING];
	memset((void*)(this->RobotStateString), 0x0, SIZE_OF_ROBOT_STATE_STRING    * sizeof(char));

	memset((void*)(&(this->CommandData)), 0x0,                                       sizeof(FRIDataSendToKRC)    );
	memset((void*)(&(this->ReadData)), 0x0,                                       sizeof(FRIDataReceivedFromKRC)  );

	pthread_mutex_init(&(this->MutexForControlData              ), NULL);
	pthread_mutex_init(&(this->MutexForThreadCreation           ), NULL);

	pthread_cond_init(&(this->CondVarForDataReceptionFromKRC    ), NULL);
	pthread_cond_init(&(this->CondVarForThreadCreation          ), NULL);

	// Thread configuration

	// get default information

	// set thread attributes to default values:
	//		detachstate			PTHREAD_CREATE_JOINABLE
	//		schedpolicy			PTHREAD_INHERIT_SCHED
	//		schedparam			Inherited from parent thread
	//		contentionscope		PTHREAD_SCOPE_SYSTEM
	//		stacksize			4K bytes
	//		stackaddr			NULL

	// Set the priorities from the initialization file.
	SchedulingParamsKRCCommunicationThread.sched_priority   =   PriorityKRCCommunicationThread;
	SchedulingParamsMainThread.sched_priority               =   PriorityMainThread;

	pthread_attr_init(&AttributesKRCCommunicationThread                                                         );

	// Set the thread scheduling policy attribute to round robin
	// default is OTHER which equals RR in QNX 6.5.0.
	pthread_attr_setschedpolicy(&AttributesKRCCommunicationThread,   SCHED_FIFO                              );

	// Set the thread's inherit-scheduling attribute to explicit
	// otherwise, the scheduling parameters Cannot be changed (e.g., priority)
	pthread_attr_setinheritsched(&AttributesKRCCommunicationThread,   PTHREAD_EXPLICIT_SCHED                  );

	pthread_attr_setschedparam(&AttributesKRCCommunicationThread,   &SchedulingParamsKRCCommunicationThread );

	// this is supposed to become the message thread
	// i.e. we have to set our own scheduling parameters
	this->MainThread = pthread_self();
	pthread_setschedparam(this->MainThread, SCHED_FIFO, &SchedulingParamsMainThread);

	FuntionResult   =   pthread_create(     &KRCCommunicationThread
	                                        ,   &AttributesKRCCommunicationThread
	                                        ,   &KRCCommunicationThreadMain
	                                        ,   this);

	if (FuntionResult != EOK)
	{
		this->OutputConsole->printf("FastResearchInterface::FastResearchInterface(): ERROR, could not start the KRC communication thread (Result: %d).\n", FuntionResult);
		getchar();
		exit(EXIT_FAILURE); // terminates the process
	}

	pthread_mutex_lock(&(this->MutexForThreadCreation));

	while (!ThreadCreated)
	{
		pthread_cond_wait (&(this->CondVarForThreadCreation), &(this->MutexForThreadCreation));
	}

	ThreadCreated   =   false;
	pthread_mutex_unlock(&(this->MutexForThreadCreation));
}


// ****************************************************************
// Destructor
//
FastResearchInterface::~FastResearchInterface()
{
	int ResultValue     =   0;

	// Bad workaround for the KUKA friUDP class, which unfortunately
	// does not use select() for sockets in order to enable timeouts.
	//
	// For the case, no UDP connection to the KRC could be
	// established, we have to wake up the communication thread
	// by sending a signal as it is still waiting for a message
	// from the KRC.

	pthread_mutex_lock(&(this->MutexForControlData));
	this->NewDataFromKRCReceived = false;
	pthread_mutex_unlock(&(this->MutexForControlData));

	delay(100 + 2 * (unsigned int)(this->CycleTime));

	pthread_mutex_lock(&(this->MutexForControlData));
	if (this->NewDataFromKRCReceived)
	{
		pthread_mutex_unlock(&(this->MutexForControlData));

		if (this->IsMachineOK())
		{
			this->StopRobot();
		}

		// The KRL program running on the robot controller waits for a
		// integer value change at position 15 (i.e., 16 in KRL).
		// A value of 30 or 40 (depending on the value of
		// this->GetKRLIntValue(14), that is, $FRI_TO_INT[15]
		// lets the KRL program call friClose() and
		// terminate itself.

		this->SetKRLIntValue(15, this->GetKRLIntValue(14));

		// wait for the next data telegram of the KRC unit
		pthread_mutex_lock(&(this->MutexForControlData));
		this->NewDataFromKRCReceived    =   false;
		pthread_mutex_unlock(&(this->MutexForControlData));
		ResultValue =   this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::~FastResearchInterface(): ERROR, the KRC unit does not respond anymore. Probably the FRI was closed or there is no UDP connection anymore.");
			getchar();
			pthread_kill(this->KRCCommunicationThread, SIGTERM);
		}

		// wait until the KRL program ended
		// (no UDP communication possible anymore)
		delay(800);

		// End the KRC communication thread
		// pthread_mutex_unlock(&(this->MutexForControlData));
		// The UDP communication is working --> normal shutdown
		this->KRCCommunicationThreadIsRunning = false;
		// pthread_mutex_unlock(&(this->MutexForControlData));
	}
	else
	{
		// No UDP communication to KRC and the communication
		// thread is blocked --> we wake him up manually
		this->KRCCommunicationThreadIsRunning = false;
		pthread_mutex_unlock(&(this->MutexForControlData));
		pthread_kill(this->KRCCommunicationThread, SIGTERM);
	}

	pthread_join(this->KRCCommunicationThread, NULL);

	delete[]    this->RobotStateString;
	delete      this->OutputConsole;
}


// ****************************************************************
// printf()
//
int FastResearchInterface::printf(const char* Format,...)
{
	int Result      =   0;

	va_list ListOfArguments;

	va_start(ListOfArguments, Format);

	Result = this->OutputConsole->printf(Format, ListOfArguments);

	va_end(ListOfArguments);

	return(Result);
}


void FastResearchInterface::setCycleTime(float time) {
	this->CycleTime = time;
}

int FastResearchInterface::GetControlScheme() const {
	return CurrentControlScheme;
}
