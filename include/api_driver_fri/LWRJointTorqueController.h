//  ---------------------- Doxygen info ----------------------
//! \file LWRJointTorqueController.h
//!
//! \brief
//! Header file for the class LWRJointTorqueController
//!
//! \details
//! The class LWRJointTorqueController constitutes an access
//! easy-to-use joint impedance control interface through the KUKA
//! Fast Research Interface of the Light-Weight Robot IV.
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


#ifndef __LWRJointTorqueController__
#define __LWRJointTorqueController__


#include <FastResearchInterface.h>
#include <LWRBaseControllerInterface.h>


//  ---------------------- Doxygen info ----------------------
//! \class LWRJointTorqueController
//!
//! \brief
//! Joint impedance controller interface for the KUKA Light-Weight
//! Robot IV
//  ----------------------------------------------------------

class LWRJointTorqueController : public LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRJointTorqueController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
//!
//! \brief
//! Constructor
//!
//! \copydetails LWRBaseControllerInterface::LWRBaseControllerInterface()
//  ----------------------------------------------------------
	LWRJointTorqueController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRJointTorqueController(void)
//!
//! \brief
//! Destructor
//!
//! \copydetails LWRBaseControllerInterface::~LWRBaseControllerInterface()
//  ----------------------------------------------------------
	~LWRJointTorqueController(void)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
//!
//! \brief
//! \copybrief FastResearchInterface::StartRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StartRobot()
//  ----------------------------------------------------------
	inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
	{
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(		FastResearchInterface::JOINT_TORQUE_CONTROL
									 	,	TimeOutValueInSeconds));
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int RestartWithPositionControl(const float &TimeOutValueInSeconds	=	120.0)
//!
//! \brief
//! \copybrief FastResearchInterface::StartRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StartRobot()
//  ----------------------------------------------------------
	inline int RestartWithPositionControl(const float &TimeOutValueInSeconds	=	120.0)
	{
		float position[7];
		this->FRI->GetMeasuredJointPositions(position);
		this->FRI->SetCommandedJointPositions(position);

		float pose[6];
		this->FRI->GetMeasuredCartPose(pose);
		this->FRI->SetCommandedCartPose(pose);
		
		// start the controller and switch to command mode
		return(this->FRI->StartRobot(		FastResearchInterface::JOINT_POSITION_CONTROL
									 	,	TimeOutValueInSeconds));
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointPositions(const float *CommandedJointPositions)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointPositions()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointPositions()
//  ----------------------------------------------------------
	inline void SetCommandedJointPositions(const float *CommandedJointPositions)
	{
		return(this->FRI->SetCommandedJointPositions(CommandedJointPositions));
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointTorques(const float *CommandedJointTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointTorques()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointTorques()
//  ----------------------------------------------------------
	inline void SetCommandedJointTorques(const float *CommandedJointTorques)
	{
		return(this->FRI->SetCommandedJointTorques(CommandedJointTorques));
	}

};	// class LWRJointTorqueController

#endif
