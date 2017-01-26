/**
 * @file		FCU__FLIGHT_CONTROL__GIMBAL_CONTROL.C
 * @brief		Gimbal Control Subsystem
 * @author		Lachlan Grogan
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
 * @addtogroup FCU__FLIGHT_CTL__GIMBAL_CONTROL
 * @ingroup FCU
 * @{ */

#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_GIMBAL_CONTROL == 1U

//the structure
extern struct _strFCU sFCU;

// TODO add to defines.h
// angles for each gimballing command
#define C_FCU__PUSHER_INTERLOCK_STATIC_TILT	0.0
#define C_FCU__PUSHER_INTERLOCK_HALF_TILT	0.5
#define C_FCU__PUSHER_INTERLOCK_FULL_TILT 	2.0
#define C_FCU__NUM_GIMBALS					4U

// TODO add this to the sFCU
struct
{
	E_FCU__GIMBALS_GS_COMM eGSCommand;
	Lfloat32 f32GimbalAngle1;
	Lfloat32 f32GimbalAngle2;
	Lfloat32 f32GimbalAngle3;
	Lfloat32 f32GimbalAngle4;
}sGimbalsControl;


// add to fcu_core_enums.h in pod drive section:
// typedef enum
// {
// 	GIMBAL_NEUTRAL_LEVEL = 0U,
// 	GIMBAL_BACKWARD_LEVEL = 1U,
// 	GIMBAL_FORWARD_LEVEL = 2U
// }E_FCU__GIMBAL_LEVEL;


/***************************************************************************//**
 * @brief
 * Init any gimbal tasks
 * 
 */
void vFCU_FCTL_GIMBALS__Init(void)
{
	sFCU.sGimbalsControl.eGSCommands = GIMBALS_CTL_DO_NOTHING; // Set the commands from the ground station to DO_NOTHING at startup
}

void vFCU_FCTL_GIMBALS__Process(void)
{

	vFCU_FCTL_GIMBALS__ManualCommandsHandle();

	switch(sFCU.eMissionPhase)
	{
		case MISSION_PHASE__PUSH_INTERLOCK_PHASE:
			// get pod speed	
			Luint32 u32PodSpeed = vFCU__POD_SPEED();
			Luint16 u16HoverEngineState = u16FCU_FCTL_HOVERENGINES__Get_State();
			// if the pod speed is lower than the pod standby speed, the pod is lifted, and the pod is hovering
			if (u32PodSpeed < PODSPEED_STANDBY && sFCU.sOpStates.u8Lifted != 0U && u16HoverEngineState == E_FCU__HOVERENGINES_STATES_T.HOVERENGINES_STATE__HOVERING)
			{
				Luint8 u8Counter;
				switch(sFCU.sGimbalsControl.eGSCommands)
				{
					case GIMBAL_CTL_M_SET_STATIC:
						// TODO crate SetAngle function in the low level gimbal driver
						vFCU__GIMBALS__SetAngle(C_FCU__PUSHER_INTERLOCK_HALF_TILT * -1);
						break;
					case GIMBAL_CTL_M_SET_FULL_BACKWARDS:
						// TODO crate SetAngle function in the low level gimbal driver
						vFCU__GIMBALS__SetAngle(C_FCU__PUSHER_INTERLOCK_FULL_TILT * -1);
						break;
				}
			}
			break;
		case MISSION_PHASE__FLIGHT_MODE:
			vFCU__GIMBALS__SetAngle(C_FCU__PUSHER_INTERLOCK_STATIC_TILT);
			break;
	}	
}

void vFCU_FCTL_GIMBALS__ManualCommandsHandle(void)
{
	switch(sFCU.sGimbalsControl.eGSCommands)
	{
		// TODO add any other commands that may be needed outside a phase
		default:
		break;
	}
}


#endif //C_LOCALDEF__LCCM655__ENABLE_GIMBAL_CONTROL
#ifndef C_LOCALDEF__LCCM655__ENABLE_GIMBAL_CONTROL
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

