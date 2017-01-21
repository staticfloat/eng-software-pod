/**
* @file       FCU_FCTRL_HOVERENGINES_CONTROL.C
* @brief      Hover Engines Control
* @author	  Alessandro Caratelli, Marek Gutt-Mostowy
* @copyright  rLoop Inc.
*/

/////////////////////////////////////////////
///////////  WORK IN PROGRESS  //////////////
/////////////////////////////////////////////

// TO DO:
// - Verification

//Hoverengines handle
//   void   vFCU_FLIGHTCTL_HOVERENGINES__Init(void);
//   void   vFCU_FLIGHTCTL_HOVERENGINES__Process(void);
//   void   vFCU_FLIGHTCTL_HOVERENGINES__enable(void);
//   void   vFCU_FLIGHTCTL_HOVERENGINES__disable(void);
//   void   vFCU_FLIGHTCTL_HOVERENGINES__start(void);
//   void   vFCU_FLIGHTCTL_HOVERENGINES__stop(void);
//   Luint16 u16FCU_FLIGHTCTL_HOVERENGINES__Get_State(Lint8 u8Enable);
//   Luint16 u16FCU_FLIGHTCTL_HOVERENGINES__Get_FaultFlag(void);


#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_HOVERENGINES_CONTROL == 1U

extern struct _strFCU sFCU;

// TODO: need values for following constants and move to fcu_core_defines.h

/** Hover Engines Parameters */
#define C_FCU__HE_STATIC_HOVER_RPM						(2000U)   // hover engine nominal RPM speed
#define C_FCU__HE_CRUISE_RPM							(2000U)   // hover engine cruise RPM speed
#define C_FCU__HE_MAX_RPM_IN_HOVERING					(2500U)   // hover engine maximum allowed RPM speed douring hovering
#define C_FCU__HE_MIN_RPM_IN_HOVERING					(1500U)   // hover engine minimum allowed RPM speed douring hovering
#define C_FCU__HE_MIN_RPM_IN_STANDBY					(10U)     // hover engine OFF state tolerance
#define C_FCU__HE_MAX_CURRENT							(10U)     // hover engine maximum allowed current TO BE CHECKED IF EXISTS
#define C_FCU__HE_MIN_CURRENT							(1U)      // hover engine minimum allowed current
#define C_FCU__HE_MAX_TEMPERATURE						(95U)     // critical hover engine temperature
/** Pod Dynamics Parameters */
#define C_FCU__XXXXX_PODSPEED_STANDBY					(5U)      // Pod standby speed
#define C_FCU__XXXXX_PODSPEED_MAX_SPEED_TO_STABILIZE	(100000U) // max Pod speed to stabilize pod / ATM extra large value so that it's never reached and the hover engines don't throttle down


#if C_LOCALDEF__LCCM655__ENABLE_HOVERENGINES_CONTROL == 1U
struct
{
	/** The hover engines state machine */
	E_FCU_HOVERENGINES__STATES_T eState;

	/** The hover engines input commands from GS */
	E_FCU_HOVERENGINES__COMMANDS_T u16HoverEnginesCommands;

	/** The hover engines RMP values from GS */
	Lint32 u32HoverEnginesRPM_Commands;

	/** Internal parameters */
	struct
	{
		Luint8 u8Enable;
		Luint8 u8RunAuto;
		Luint8 u8SpeedState;
	
	} sIntParams

}sHoverengines
#endif

typedef enum
{
	HOVERENGINES_STATE__IDLE = 0U,
	HOVERENGINES_STATE__ENABLED,
	HOVERENGINES_STATE__START_STATIC_HOVERING,
	HOVERENGINES_STATE__STATIC_HOVERING

} E_FCU__HOVERENGINES__STATES_T;

typedef enum
{
	DO_NOTHING = 0U,
	STATIC_HOVERING,
	RELEASE_STATIC_HOVERING,
	M_SET_SPEED_HE1,
	M_SET_SPEED_HE2,
	M_SET_SPEED_HE3,
	M_SET_SPEED_HE4,
	M_SET_SPEED_HE5,
	M_SET_SPEED_HE6,
	M_SET_SPEED_HE7,
	M_SET_SPEED_HE8

} E_FCU__HOVERENGINES_COMMANDS;


// TODO: need the following functions:
// vFCU_COOLING__Set_Valve(ValveNumber, TimeOn, TimeOff);
// vFCU_COOLING__Set_Valve(ValveNumber, TimeOn, TimeOff);
// vFCU_COOLING__Set_Valve(ValveNumber, TimeOn, TimeOff); 
// vFCU_COOLING__Set_Valve(ValveNumber, TimeOn, TimeOff);
// u32FCU_FLIGHTCTL_XXXXX__PodSpeed();

void vFCU_FLIGHTCTL_HOVERENGINES__Init(void)
{

	sFCU.sHoverengines.sIntParams.u8Enable = 0U; // temporary variable to be used inside of the functions
	sFCU.sHoverengines.sIntParams.u8SpeedState = 0U; //TO BE LOOKED UP AGAIN
	sFCU.sHoverengines.sIntParams.u8RunAuto = 0U; // Flag to initiate flight mode
	sFCU.sHoverengines.sIntParams.u8TempVar = 0U; // Tempo variable used inside of the functions

	sFCU.sHoverengines.u16HoverenginesCommands = DO_NOTHING; // Set the commands from the ground station to DO_NOTHING at startup

	sFCU.sHoverengines.eState = HOVERENGINES_STATE__IDLE; // set the first state of the hover engines control state machine to IDLE

	vFCU_THROTTLE__Enable_Run(); // Enable the throttles state machine
}


void vFCU_FLIGHTCTL_HOVERENGINES__Process(void)
{
	Luint32 u32PodSpeed; // SHOULD COME FROM THE NAVIGATION FUNCTIONS
	Luint16 u16RPM[8]; // Used in order to avoid fetching from structure


	vFCU_FLIGHTCTL_HOVERENGINES__ManualCommandsHandle(); // Call the function with manual commands handling

	switch(sFCU.sHoverengines.eState)
	{

		case HOVERENGINES_STATE__IDLE:
			// this is the idle state: Hover Engines are disabled.
			// if is received the "Enable hover engines command"
			// or if is active the autonomous behaviour set by the TOD_DRIVE FSM
			// and the pod speed is lower than the pod standby speed
			// than we enable the Hover Engines and we move to the next state
			if((sFCU.sHoverengines.u16HoverenginesCommands == ENABLE_HE) ||
			   (sFCU.sHoverengines.sIntParams.u8Enable == 1U && sFCU.sHoverengines.sIntParams.u8RunAuto == 1U))
			{
				u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__PodSpeed();
				if(u32PodSpeed < PODSPEED_STANDBY)
				{
					Luint8 u8Counter = 0;
					for(u8Counter = 1; u8Counter < 8; u8Counter++)
					{
						vFCU_THROTTLE__Enable(u8Counter);  // This function is not yet implemented
					}
					sFCU.sHoverengines.sIntParams.u8TempVar = 0U;
					sFCU.sHoverengines.eState = HOVERENGINES_STATE__ENABLED;
				}
			}
			break;


		case HOVERENGINES_STATE__ENABLED:
			// In this state the Hover Engines are enabled.
			// if is received the "Static hovering" command
			// or if is active the autonomous behaviour set by the POD_DRIVE FSM
			// and the pod speed is lower than the pod standby speed
			// than, for the first group of hoverengines, the speed is linearly
			// set from 0 to the hover engine nominal RPM
			// switch state only when the measured RPM reach
			// the HOVER_ENGINE_STATIC_HOVER_RPM with a certain tollerance
			Lint16 status = 0;

			if(sFCU.sHoverengines.u16HoverenginesCommands == DISABLE_HE)
			{
				Luint8 u8Counter = 0;
				for(u8Counter = 1; u8Counter < 8; u8Counter++)
				{
					vFCU_THROTTLE__Disable(u8Counter);  // This function is not yet implemented
				}
				sFCU.sHoverengines.eState = HOVERENGINES_STATE__IDLE;
			}

			if((sFCU.sHoverengines.u16HoverenginesCommands == STATIC_HOVERING) ||
			   (sFCU.sHoverengines.sIntParams.u8Enable == 1U && sFCU.sHoverengines.sIntParams.u8RunAuto == 1U)) //If we are hovering and HEs are enabled and we are in autonomous flight mode 
			{
				u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__PodSpeed(); //read the pod speed
				if(sFCU.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY) && sFCU.sHoverengines.sIntParams.u8TempVar == 0U) // comparison value to be defined
				{
					vFCU_COOLING__Set_Valve(1, 0.5, 1.5); // to be changed (this function is not yet implemented
					vFCU_COOLING__Set_Valve(2, 0.5, 1.5); // to be changed (this function is not yet implemented
					vFCU_COOLING__Set_Valve(5, 0.5, 1.5); // to be changed (this function is not yet implemented
					vFCU_COOLING__Set_Valve(6, 0.5, 1.5); // to be changed (this function is not yet implemented
					vFCU_THROTTLE__Set_Throttle(1, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
					vFCU_THROTTLE__Set_Throttle(2, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
					vFCU_THROTTLE__Set_Throttle(5, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
					vFCU_THROTTLE__Set_Throttle(6, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
					sFCU.sHoverengines.sIntParams.u8TempVar = 1U;
				}
			}

			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[1]));
			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[2]));
			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[5]));
			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[6]));
			if (status != 0)
			{
				// TO DO
			}
			else
				if((u16RPM[1] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING) &&
				   (u16RPM[2] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING) &&
				   (u16RPM[5] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING) &&
				   (u16RPM[6] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING))
				{
					sFCU.sHoverengines.eState = HOVERENGINES_STATE__START_STATIC_HOVERING;
				}
			break;


		case HOVERENGINES_STATE__START_STATIC_HOVERING:
			// In this state HE speed for the 1th group was set.
			// for the second group of hoverengines, the speed is linearly
			// set from 0 to the hover engine nominal RPM
			// it switch state only when the measured RPM reach
			// the HOVER_ENGINE_STATIC_HOVER_RPM with a certain tollerance
			Lint16 status = 0;
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__PodSpeed(); //Uses a function that does not exist
			if(sFCU.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY) && sFCU.sHoverengines.sIntParams.u8TempVar < 2U) // comparison value to be defined
			{
				vFCU_COOLING__Set_Valve(3, 0.5, 1.5); // to be changed (this function is not yet implemented
				vFCU_COOLING__Set_Valve(4, 0.5, 1.5); // to be changed (this function is not yet implemented
				vFCU_COOLING__Set_Valve(7, 0.5, 1.5); // to be changed (this function is not yet implemented
				vFCU_COOLING__Set_Valve(8, 0.5, 1.5); // to be changed (this function is not yet implemented
				vFCU_THROTTLE__Set_Throttle(3, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
				vFCU_THROTTLE__Set_Throttle(4, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
				vFCU_THROTTLE__Set_Throttle(7, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
				vFCU_THROTTLE__Set_Throttle(8, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__RAMP);
				sFCU.sHoverengines.sIntParams.u8TempVar = 2U;
			}
			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[3]));
			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[4]));
			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[7]));
			status = status || s16FCU_ASI__ReadMotorRpm(devIndex, &(u16RPM[8]));
			if (status != 0)
			{
				// TO DO
			}
			else
				if((u16RPM[3] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING) &&
					 (u16RPM[4] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING) &&
					 (u16RPM[7] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING) &&
					 (u16RPM[8] >= HOVER_ENGINE_MIN_RPM_IN_HOVERING))
				{
					sFCU.sHoverengines.eState = HOVERENGINES_STATE__STATIC_HOVERING;
					sFCU.sHoverengines.sIntParams.u8RunAuto = 1U;
				}
			break;


		case HOVERENGINES_STATE__STATIC_HOVERING:
			// In this state all Hover Engines are running at the STATIC_HOVER_RPM.

			Lint16 status1 = 0;
			Lint16 status2 = 0;
			Luint8 u8Counter = 0;

			for(u8Counter = 1; u8Counter < 8; u8Counter++) // VERIFY PARAMETERS
			{
				// RPM, Temperature and Current are monitored,
				// a fault is reported if those values goes out of the safety range.
				Luint64* pu64Rpm;
				Luint64* pu64Current;
				Luint64* pu64Temp;
				s16FCU_ASI_CTRL__ReadMotorRpm(u8Counter,  &pu64Rpm);
				s16FCU_ASI_CTRL__ReadMotorCurrent(u8Counter,  &pu64Current));
				s16FCU_ASI_CTRL__ReadControllerTemperature(u8Counter, &pu64Temp);
				if((pu64Rpm > HOVER_ENGINE_MAX_RPM_IN_HOVERING) || (pu64Rpm < HOVER_ENGINE_MIN_RPM_IN_HOVERING))
					{/* Fault handle TO DO */ }
				if((pu64Current > HOVER_ENGINE_MAX_CURRENT) || (pu64Current < HOVER_ENGINE_MIN_CURRENT))
					{/* Fault handle TO DO */ }
				if(pu64Rpm > HOVER_ENGINE_MAX_TEMPERATURE)
					{/* Fault handle TO DO */ }
			}

			// If the pod speed goes higher than the max speed to stabilize pod the HE RPM is reduced to hover engine cruise RPM
			// If the pod speed goes lower than the max speed to stabilize pod the HE RPM is increased to hover engine nominal RPM
			if(sFCU.sHoverengines.sIntParams.u8Enable == 1U) //Manage HE RPM during flight
			{
				u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__PodSpeed();
				if(u32PodSpeed > PODSPEED_MAX_SPEED_TO_STABILIZE) //PODSPEED_MAX parameter to be defined
					if(sFCU.sHoverengines.sIntParams.u8SpeedState == 0U)
					{
						for(u8Counter = 1; u8Counter < 8; u8Counter++)
							vFCU_THROTTLE__Set_Throttle(u8Counter, HOVER_ENGINE_CRUISE_RPM, THROTTLE_TYPE__STEP);
						sFCU.sHoverengines.sIntParams.u8SpeedState = 1U;
					}
				else
					if(sFCU.sHoverengines.sIntParams.u8SpeedState == 1U)
					{
						for(u8Counter = 1; u8Counter < 8; u8Counter++)
							vFCU_THROTTLE__Set_Throttle(u8Counter, HOVER_ENGINE_STATIC_HOVER_RPM, THROTTLE_TYPE__STEP);
						sFCU.sHoverengines.sIntParams.u8SpeedState = 0U;
					}
			}

			// If is received the command to release the static hovering
			// or if the POD_DRIVE FSM disable the HE,
			// and if the pod speed is under the standby speed
			// the HE are set to the minimum RPM and the cooling is turned off
			if((sFCU.sHoverengines.u16HoverenginesCommands == RELEASE_STATIC_HOVERING) ||
		     (sFCU.sHoverengines.sIntParams.u8RunAuto == 0U))
			{
				u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__PodSpeed();
				if(sFCU.sOpStates.u8StaticHovering && (u32PodSpeed < PODSPEED_STANDBY)) // comparison value to be defined
				{
					for(u8Counter = 1; u8Counter < 8; u8Counter++)
					{
						vFCU_THROTTLE__Set_Throttle(u8Counter, 0, THROTTLE_TYPE__STEP);
						vFCU_COOLING__Set_Valve(u8Counter, 0.0, 2.0);
					}
				}
			}

			// It disables the throtles and switch to the idle state only when the
			// HE RPM goes to 0 with a certain tolerance
			// report eventual errors
			for(u8Counter = 1; u8Counter < 8; u8Counter++)
			{
				Lint16 u16RPM;
				status1 = status1 || s16FCU_ASI__ReadMotorRpm(devIndex, &u16RPM);
				status2 = (u16RPM <= HOVER_ENGINE_MIN_RPM_IN_STANDBY) ? 0 : 1;
			}
			if (status1 != 0)
			{
				// TO DO
			}
			else
			{
				if(!status2)
				{
					for(u8Counter = 1; u8Counter < 8; u8Counter++)
					{
						vFCU_THROTTLE__Disable(u8Counter); // This function is not yet implemented
					}
					sFCU.sHoverengines.sIntParams.u8Enable = 0U;
					sFCU.sHoverengines.sIntParams.u8RunAuto = 0U;
					sFCU.sHoverengines.eState = HOVERENGINES_STATE__IDLE;
				}
			}
			break;
	} //switch(sFCU.sHoverengines.eState)

}

Luint16 u16FCU_FLIGHTCTL_HOVERENGINES__Get_State(void)
{
	return sFCU.sHoverengines.eState;
}

Luint16 u16FCU_FLIGHTCTL_HOVERENGINES__Get_FaultFlag(void){
	// TO DO
}

void vFCU_FLIGHTCTL_HOVERENGINES__enable(void)
{
	sFCU.sHoverengines.sIntParams.u8Enable = 1U;
}

void vFCU_FLIGHTCTL_HOVERENGINES__disable(void)
{
	sFCU.sHoverengines.sIntParams.u8RunAuto = 0U;
	sFCU.sHoverengines.sIntParams.u8Enable  = 0U;
}

void vFCU_FLIGHTCTL_HOVERENGINES__start(void)
{
	if(sFCU.sHoverengines.sIntParams.u8Enable != 0U)
		sFCU.sHoverengines.sIntParams.u8RunAuto = 1U;
}

void vFCU_FLIGHTCTL_HOVERENGINES__stop(void)
{
	sFCU.sHoverengines.sIntParams.u8RunAuto  = 0U;
}


void vFCU_FLIGHTCTL_HOVERENGINES__ManualCommandsHandle(void)
{
	Luint8 u8ManualControlActive;
	Luint32 u32PodSpeed;
	Luint32 u32GS_RPM = sFCU.sHoverengines.u32HoverenginesRPM_Commands;
	switch(sFCU.sHoverengines.u16HoverenginesCommands)
	{
		case M_SET_SPEED_HE1:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(1U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;

		case M_SET_SPEED_HE2:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(2U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;

		case M_SET_SPEED_HE3:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(3U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;

		case M_SET_SPEED_HE4:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(4U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;

		case M_SET_SPEED_HE5:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(5U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;

		case M_SET_SPEED_HE6:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(6U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;

		case M_SET_SPEED_HE7:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(7U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;

		case M_SET_SPEED_HE8:
			u32PodSpeed = u32FCU_FLIGHTCTL_XXXXX__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(8U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;
	}
}



#endif //C_LOCALDEF__LCCM655__ENABLE_HOVERENGINES_CONTROL
#ifndef C_LOCALDEF__LCCM655__ENABLE_HOVERENGINES_CONTROL
#error
#endif

#endif //C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL
#ifndef C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL
#error
#endif

#endif //#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
//safetys
#ifndef C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE
#error
#endif
