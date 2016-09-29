//  ---------------------- Doxygen info ----------------------
//! \file LWRBaseControllerInterface.h
//!
//! \brief
//! Header file for the class LWRBaseControllerInterface
//!
//! \details
//! The class LWRBaseControllerInterface constitutes the base class
//! for the actual control interface classes:
//!
//!  - class LWRJointPositionController
//!  - class CartImpedanceController
//!  - class LWRJointImpedanceController
//!  - class LWRJointTorqueController
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


#ifndef __LWRBaseControllerInterface__
#define __LWRBaseControllerInterface__

#include <FastResearchInterface.h>
#include <FRICommunication.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <stdarg.h>



//  ---------------------- Doxygen info ----------------------
//! \class LWRBaseControllerInterface
//!
//! \brief
//! Base class for other controller interface classes of the
//! KUKA Fast Research Interface for the Light-Weight Robot IV
//!
//! \details
//! This class constitutes the base class for the four different controller interface classes, which represent the
//! actual user API for the Fast Research Library:
//! <ul>
//! <li>LWRJointPositionController for the joint position controller,
//! <li>LWRCartImpedanceController for the Cartesian impedance controller,
//! <li>LWRJointImpedanceController for the joint impedance controller, and
//! <li>LWRJointTorqueController for the joint torque controller,\n\n
//! </ul>
//! This base class only features one attribute, which is a pointer to an object of the class FastResearchInterface.
//! Most of the functionality of the class FastResearchInterface remains hidden, and only a small subset that is
//! actually necessary for convenient user API is used in this base class and it derivatives.
//!
//! \sa FastResearchInterface
//! \sa LWRJointPositionController
//! \sa LWRCartImpedanceController
//! \sa LWRJointImpedanceController
//! \sa LWRJointTorqueController
//  ----------------------------------------------------------

class LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRBaseControllerInterface(const char *InitFileName)
//!
//! \brief
//! Constructor
//!
//! \details
//! The constructor creates the actual object of the class FastResearchInterface, which is used by this class.
//!
//! \copydetails FastResearchInterface::FastResearchInterface()
//  ----------------------------------------------------------
	LWRBaseControllerInterface()
	{
		this->FRI           =   new FastResearchInterface();
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRBaseControllerInterface(void)
//!
//! \brief
//! Destructor
//!
//! \details
//! The destructor deletes the actual object of the class FastResearchInterface, which is used by this class.
//!
//! \copydetails FastResearchInterface::~FastResearchInterface()
//  ----------------------------------------------------------
	~LWRBaseControllerInterface()
	{
		delete this->FRI;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn virtual inline int StartRobot(const float &TimeOutValueInSeconds) = 0
//!
//! \brief
//! \copybrief FastResearchInterface::StartRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StartRobot()
//  ----------------------------------------------------------
	inline int StartRobotInJointPositionControl(const float &TimeOutValueInSeconds = 20.f) {
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(      FastResearchInterface::JOINT_POSITION_CONTROL
		                                   ,  TimeOutValueInSeconds));
	}

	inline int StartRobotInJointImpedanceControl(const float &TimeOutValueInSeconds = 20.f) {
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(      FastResearchInterface::JOINT_IMPEDANCE_CONTROL
		                                   ,  TimeOutValueInSeconds));
	}

	inline int StartRobotInCartesianImpedanceControl(const float &TimeOutValueInSeconds = 20.f) {
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(      FastResearchInterface::CART_IMPEDANCE_CONTROL
		                                   ,  TimeOutValueInSeconds));
	}

	inline int StartRobotInJointTorqueControl(const float &TimeOutValueInSeconds = 20.f) {
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(      FastResearchInterface::JOINT_TORQUE_CONTROL
		                                   ,  TimeOutValueInSeconds));
	}

	inline int StartRobotInJointDynamicControl(const float &TimeOutValueInSeconds = 20.f) {
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(      FastResearchInterface::JOINT_DYNAMIC_CONTROL
		                                   ,  TimeOutValueInSeconds));
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int StopRobot(void)
//!
//! \brief
//! \copybrief FastResearchInterface::StopRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StopRobot()
//  ----------------------------------------------------------
	inline int StopRobot(void)
	{
		return(this->FRI->StopRobot());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMeasuredJointPositions(float *MeasuredJointPositions)
//!
//! \brief
//! \copybrief FastResearchInterface::GetMeasuredJointPositions()
//!
//! \details
//! \copydetails FastResearchInterface::GetMeasuredJointPositions()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetMeasuredJointPositions(float *MeasuredJointPositions)
	{
		this->FRI->GetMeasuredJointPositions(MeasuredJointPositions);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetCommandedJointPositions(float *CommandedJointPositions)
//!
//! \brief
//! \copybrief FastResearchInterface::GetCommandedJointPositions()
//!
//! \details
//! \copydetails FastResearchInterface::GetCommandedJointPositions()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetCommandedJointPositions(float *CommandedJointPositions)
	{
		this->FRI->GetCommandedJointPositions(CommandedJointPositions);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetCommandedJointPositionOffsets(float *CommandedJointPositionOffsets)
//!
//! \brief
//! \copybrief FastResearchInterface::GetCommandedJointPositionOffsets()
//!
//! \details
//! \copydetails FastResearchInterface::GetCommandedJointPositionOffsets()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetCommandedJointPositionOffsets(float *CommandedJointPositionOffsets)
	{
		this->FRI->GetCommandedJointPositionOffsets(CommandedJointPositionOffsets);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMeasuredJointTorques(float *MeasuredJointTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::GetMeasuredJointTorques()
//!
//! \details
//! \copydetails FastResearchInterface::GetMeasuredJointTorques()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetMeasuredJointTorques(float *MeasuredJointTorques)
	{
		this->FRI->GetMeasuredJointTorques(MeasuredJointTorques);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetEstimatedExternalJointTorques(float *EstimatedExternalJointTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::GetEstimatedExternalJointTorques()
//!
//! \details
//! \copydetails FastResearchInterface::GetEstimatedExternalJointTorques()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetEstimatedExternalJointTorques(float *EstimatedExternalJointTorques)
	{
		this->FRI->GetEstimatedExternalJointTorques(EstimatedExternalJointTorques);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMeasuredCartPose(float *MeasuredCartPose)
//!
//! \brief
//! \copybrief FastResearchInterface::GetMeasuredCartPose()
//!
//! \details
//! \copydetails FastResearchInterface::GetMeasuredCartPose()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetMeasuredCartPose(float *MeasuredCartPose)
	{
		this->FRI->GetMeasuredCartPose(MeasuredCartPose);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetCommandedCartPose(float *CommandedCartPose)
//!
//! \brief
//! \copybrief FastResearchInterface::GetCommandedCartPose()
//!
//! \details
//! \copydetails FastResearchInterface::GetCommandedCartPose()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>-------------------------------
	inline int GetCommandedCartPose(float *CommandedCartPose)
	{
		this->FRI->GetCommandedCartPose(CommandedCartPose);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetCommandedCartPoseOffsets(float *CommandedCartPoseOffsets)
//! \brief
//! \copybrief FastResearchInterface::GetCommandedCartPoseOffsets()
//!
//! \details
//! \copydetails FastResearchInterface::GetCommandedCartPoseOffsets()
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit exists.
//! <li> \c EOK if no error occurred.
//! </ul>
//  ----------------------------------------------------------
	inline int GetCommandedCartPoseOffsets(float *CommandedCartPoseOffsets)
	{
		this->FRI->GetCommandedCartPoseOffsets(CommandedCartPoseOffsets);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}

//  ---------------------- Doxygen info ----------------------
//! \fn void GetCurrentMassMatrix(float **MassMatrix)
//!
//! \brief
//! \copybrief FastResearchInterface::GetCurrentMassMatrix()
//!
//! \details
//! \copydetails FastResearchInterface::GetCurrentMassMatrix()
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	int GetCurrentMassMatrix(float **MassMatrix) {
		this->FRI->GetCurrentMassMatrix(MassMatrix);
		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCurrentGravityVector(float *GravityVector)
//!
//! \brief
//! \copybrief FastResearchInterface::GetCurrentGravityVector()
//!
//! \details
//! \copydetails FastResearchInterface::GetCurrentGravityVector()
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	int GetCurrentGravityVector(float *GravityVector) {
		this->FRI->GetCurrentGravityVector(GravityVector);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
//!
//! \brief
//! \copybrief FastResearchInterface::WaitForKRCTick()
//!
//! \details
//! \copydetails FastResearchInterface::WaitForKRCTick()
//  ----------------------------------------------------------
	inline int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
	{
		return(this->FRI->WaitForKRCTick(TimeoutValueInMicroSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool IsMachineOK(void)
//!
//! \brief
//! \copybrief FastResearchInterface::IsMachineOK()
//!
//! \details
//! \copydetails FastResearchInterface::IsMachineOK()
//  ----------------------------------------------------------
	inline bool IsMachineOK(void)
	{
		return(this->FRI->IsMachineOK());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline float GetCycleTime(void)
//!
//! \brief
//! \copybrief FastResearchInterface::GetFRICycleTime()
//!
//! \details
//! \copydetails FastResearchInterface::GetFRICycleTime()
//  ----------------------------------------------------------
	inline float GetCycleTime(void)
	{
		return(this->FRI->GetFRICycleTime());
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline const char* GetCompleteRobotStateAndInformation(void)
//!
//! \brief
//! \copybrief FastResearchInterface::GetCompleteRobotStateAndInformation()
//!
//! \details
//! \copydetails FastResearchInterface::GetCompleteRobotStateAndInformation()
//  ----------------------------------------------------------
	inline const char* GetCompleteRobotStateAndInformation(void)
	{
		return(FRI->GetCompleteRobotStateAndInformation());
	}

//  ---------------------- Doxygen info ----------------------
//! \fn inline int printf(const char* Format, ...)
//!
//! \brief
//! \copybrief FastResearchInterface::printf()
//!
//! \details
//! \copydetails FastResearchInterface::printf()
//  ----------------------------------------------------------
	inline int printf(const char* Format, ...)
	{
		int Result      =   0;
		va_list ListOfArguments;

		va_start(ListOfArguments, Format);
		Result = FRI->printf(Format, ListOfArguments);
		va_end(ListOfArguments);

		return(Result);
	}

	void setCycleTime(float time) {
		FRI->setCycleTime(time);
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
		if( this->FRI->GetControlScheme() == FastResearchInterface::JOINT_POSITION_CONTROL or
		    this->FRI->GetControlScheme() == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
		{
			this->FRI->SetCommandedJointPositions(CommandedJointPositions);
		}
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
		if( this->FRI->GetControlScheme() == FastResearchInterface::JOINT_IMPEDANCE_CONTROL or
		    this->FRI->GetControlScheme() == FastResearchInterface::JOINT_TORQUE_CONTROL or
		    this->FRI->GetControlScheme() == FastResearchInterface::JOINT_DYNAMIC_CONTROL)
		{
			this->FRI->SetCommandedJointTorques(CommandedJointTorques);
		}
	}


	//  ---------------------- Doxygen info ----------------------
	//! \fn inline void SetCommandedJointStiffness(const float *CommandedJointStiffness)
	//!
	//! \brief
	//! \copybrief FastResearchInterface::SetCommandedJointStiffness()
	//!
	//! \details
	//! \copydetails FastResearchInterface::SetCommandedJointStiffness()
	//  ----------------------------------------------------------
	inline void SetCommandedJointStiffness(const float *CommandedJointStiffness)
	{
		if( this->FRI->GetControlScheme() == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
		{
			this->FRI->SetCommandedJointStiffness(CommandedJointStiffness);
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointDamping(const float *CommandedJointDamping)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointDamping()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointDamping()
//  ----------------------------------------------------------
	inline void SetCommandedJointDamping(const float *CommandedJointDamping)
	{
		if( this->FRI->GetControlScheme() == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
		{
			this->FRI->SetCommandedJointDamping(CommandedJointDamping);
		}
	}

	//  ---------------------- Doxygen info ----------------------
	//! \fn inline void SetCommandedCartPose(const float *CommandedCartPose)
	//!
	//! \brief
	//! \copybrief FastResearchInterface::SetCommandedCartPose()
	//!
	//! \details
	//! \copydetails FastResearchInterface::SetCommandedCartPose()
	//  ----------------------------------------------------------
	inline void SetCommandedCartPose(const float *CommandedCartPose)
	{
		if( this->FRI->GetControlScheme() == FastResearchInterface::CART_IMPEDANCE_CONTROL)
		{
			this->FRI->SetCommandedCartPose(CommandedCartPose);
		}
	}


	//  ---------------------- Doxygen info ----------------------
	//! \fn inline void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
	//!
	//! \brief
	//! \copybrief FastResearchInterface::SetCommandedCartForcesAndTorques()
	//!
	//! \details
	//! \copydetails FastResearchInterface::SetCommandedCartForcesAndTorques()
	//  ----------------------------------------------------------
	inline void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
	{
		if( this->FRI->GetControlScheme() == FastResearchInterface::CART_IMPEDANCE_CONTROL)
		{
			this->FRI->SetCommandedCartForcesAndTorques(CartForcesAndTorques);
		}
	}


	//  ---------------------- Doxygen info ----------------------
	//! \fn inline void SetCommandedCartStiffness(const float *CommandedCartStiffness)
	//!
	//! \brief
	//! \copybrief FastResearchInterface::SetCommandedCartStiffness()
	//!
	//! \details
	//! \copydetails FastResearchInterface::SetCommandedCartStiffness()
	//  ----------------------------------------------------------
	inline void SetCommandedCartStiffness(const float *CommandedCartStiffness)
	{
		if( this->FRI->GetControlScheme() == FastResearchInterface::CART_IMPEDANCE_CONTROL)
		{
			this->FRI->SetCommandedCartStiffness(CommandedCartStiffness);
		}
	}


	//  ---------------------- Doxygen info ----------------------
	//! \fn inline void SetCommandedCartDamping(const float *CommandedCartDamping)
	//!
	//! \brief
	//! \copybrief FastResearchInterface::SetCommandedCartDamping()
	//!
	//! \details
	//! \copydetails FastResearchInterface::SetCommandedCartDamping()
	//  ----------------------------------------------------------
	inline void SetCommandedCartDamping(const float *CommandedCartDamping)
	{
		if( this->FRI->GetControlScheme() == FastResearchInterface::CART_IMPEDANCE_CONTROL)
		{
			this->FRI->SetCommandedCartDamping(CommandedCartDamping);
		}
	}


	//  ---------------------- Doxygen info ----------------------
	//! \fn inline void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
	//!
	//! \brief
	//! \copybrief FastResearchInterface::GetEstimatedExternalCartForcesAndTorques()
	//!
	//! \details
	//! \copydetails FastResearchInterface::GetEstimatedExternalCartForcesAndTorques()
	//  ----------------------------------------------------------
	inline int GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
	{
		this->FRI->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesAndTorques);

		if (this->FRI->GetFRIMode() == FRI_STATE_OFF)
		{
			return(ENOTCONN);
		}
		else
		{
			return(EOK);
		}
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
	inline int RestartWithPositionControl(const float &TimeOutValueInSeconds    =   120.0)
	{
		float position[7], offsets[7];
		this->FRI->GetMeasuredJointPositions(position);
		this->FRI->GetCommandedJointPositionOffsets(offsets);
		for (size_t i = 0; i < 7; i++) {
			position[i] += offsets[i];
		}
		this->FRI->SetCommandedJointPositions(position);

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(       FastResearchInterface::JOINT_POSITION_CONTROL
		                                    ,   TimeOutValueInSeconds));
	}

protected:


//  ---------------------- Doxygen info ----------------------
//! \var FastResearchInterface *FRI
//!
//! \brief
//! A pointer to the actual object of the class FastResearchInterface
//  ----------------------------------------------------------
	FastResearchInterface       *FRI;

};  // class LWRBaseControllerInterface

#endif
