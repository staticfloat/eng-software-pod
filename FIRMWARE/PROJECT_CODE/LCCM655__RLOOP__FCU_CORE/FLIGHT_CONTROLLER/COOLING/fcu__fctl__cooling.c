/**
 * @file		FCU__FLIGHT_CONTROL__COOLING_CONTROL.C
 * @brief		Cooling Control Subsystem
 * @author		Paul Le Henaff
 * @copyright	rLoop Inc.
 */
/**
 * @addtogroup RLOOP
 * @{ */
/**
 * @addtogroup FCU
 * @ingroup RLOOP
 * @{ */
/**
 * @addtogroup FCU__FLIGHT_CTL__COOLING_CONTROL
 * @ingroup FCU
 * @{ */
/**
Inputs
=======
operating mode
speed
hover engine 1 temperature, ..., hover engine 8 temperature
stepper motor left temperature, stepper motor right temperature
PV pressure sensor value

Outputs
========
solenoid valve 1 command, ..., solenoid valve 6 command
*/
#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_COOLING_CONTROL == 1U

//the structure
extern struct _strFCU sFCU;

// TODO change this to a more scientific valid value?
#define C_FCU__COOLING__HE_GROUP_STARTUP_DELAY 5U
#define C_FCU__COOLING__SOLENOID_DUTYCYCLE_ON 500 //ms
#define C_FCU__COOLING__SOLENOID_DUTYCYCLE_OFF 1500 //ms

struct
{
	E_FCU__COOLING_GS_COMM eGSCommand;

		/** The hover engines state machine */
	E_FCU_COOLING__STATES_T eState;

	/** The hover engines input commands from GS */
	E_FCU_COOLING__COMMANDS_T u16CoolingCommands;

	Luint8 u8HESolenoid1;
	Luint8 u8HESolenoid2;
	Luint8 u8HESolenoid3;
	Luint8 u8HESolenoid4;
	Luint8 u8HESolenoid5;
	Luint8 u8HESolenoid6;
	Luint8 u8HESolenoid7;
	Luint8 u8HESolenoid8;
}sCoolingControl;

typedef enum
{
	COOLING_STATE__DISABLED,
	COOLING_STATE__ENABLED
} E_FCU_COOLING__STATES_T;

typedef enum
{
	COOLING_COMMAND__,
	COOLING_STATE__ENABLED,
	COOLING_STATE__START_STATIC_HOVERING,
	COOLING_STATE__STATIC_HOVERING

} E_FCU_COOLING__COMMANDS_T;

void vFCU_FCTL_COOLING__Init(void)
{
	sFCU.sCoolingControl.eGSCommands = GIMBALS_CTL_DO_NOTHING; // Set the commands from the ground station to DO_NOTHING at startup
}



void vFCU_FCTL_COOLING__Process(void)
{
	vFCU_FCTL_COOLING__ManualCommandsHandle();
	

	// switch(sFCU.eMissionPhase)
	// {
	// 	case MISSION_PHASE__PUSH_INTERLOCK_PHASE:
	// 		break;
	// }
}

void vFCU_FCTL_COOLING__ManualCommandsHandle(void)
{
	Luint8 u8ManualControlActive;
	Luint32 u32PodSpeed;
	Luint32 u32GS_RPM = sFCU.sHoverengines.u32HoverenginesRPM_Commands;
	switch(sFCU.sHoverengines.u16HoverenginesCommands)
	{
		case M_SET_SOLENOID1:
			u32PodSpeed = u32FCU_FLIGHTCTL_NAV__POD_SPEED();
			if(u32PodSpeed < PODSPEED_STANDBY)
				vFCU_THROTTLE__Set_Throttle(1U, u32GS_RPM, THROTTLE_TYPE__STEP);
			break;
	}
	"Initiate Cooling System Y" from Ground Station (where Y is either 1, ..., 6), the FCU shall continuously set solenoid valve Y command to the duty cycle solenoid valve ON time | solenoid valve OFF time.
	"Stop Cooling System Y" or "Close solenoid valve Y" from Ground Station (where Y is either 1, ..., 6), the FCU shall set solenoid valve Y command to OFF.
	"Open solenoid valve Y" from Ground Station (where Y is either 1, ..., 6), the FCU shall set solenoid valve Y command to ON.

	// switch(sFCU.sCoolingControl.eGSCommands)
	// {

	// 	Luint32 u32PodSpeed = vFCU__POD_SPEED();
	// 	Luint16 u16HoverEngineState = u16FCU_FCTL_HOVERENGINES__Get_State();

	// 	if ((sFCU.eMissionPhase == test phase ||
	// 		sFCU.eMissionPhase == pre-run phase ||
	// 		sFCU.eMissionPhase == post-run phase) &&
	// 		sFCU.sOpStates.u8Lifted == 1U &&
	// 		u32PodSpeed < PODSPEED_STANDBY)
	// 	{

	// 	}
	// 	// case:
	// 	// Upon reception of Command Static Hovering from Ground Station, if the mission phase 
	// 	// is test phase, pre-run phase or post-run phase, if lifted state is true and if the 
	// 	// speed is below standby speed, the FCU shall do the following actions, first for 
	// 	// the {HE1, HE2, HE5, HE6} group, then for the {HE3, HE4, HE7, HE8} group, with 
	// 	// HE group start-up delay (to avoid high current load at start-up):
	// 	// start the HE cooling system for this group of HE by continuously setting the corresponding solenoid valve commands to the duty cycle solenoid valve ON time | solenoid valve OFF time (0.5s / 1.5s, reference: Thermal)
	// 	// (actuation of HE is specified in 7. Control Hover Engines)
	// }
}

void vFCU_COOLING__Set_Valve(Luint8 valveNumber, Luint8 timeOn, Luint8 timeOff);
{

}



#endif //C_LOCALDEF__LCCM655__ENABLE_COOLING_CONTROL
#ifndef C_LOCALDEF__LCCM655__ENABLE_COOLING_CONTROL
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
/** @} */
/** @} */
/** @} */

