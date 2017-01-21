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

struct
{
	E_FCU__GIMBAL_GS_COMM eGSCommand;
}sGimbalControl;

void vFCU_FLIGHTCTL_GIMBAL__Init(void)
{

}

//TODO: need the following functions:
//vFCU_FLIGHTCTL_GIMBAL__SetAngle(DEGREES)

void vFCU_FLIGHTCTL_GIMBAL__Process(void)
{

	switch(sFCU.eMissionPhase){
		case MISSION_PHASE__PUSH_INTERLOCK_PHASE:

			// get pod speed	
			u32PodSpeed = vFCU__POD_SPEED();
		
			//get HE rpm
			u8status = 1U;
			for(u8Counter = 1U; u8Counter < 8U; u8Counter++)
			{
					Luint16 u16Rpm = 0U;
					s16FCU_ASI__ReadMotorRpm(u8Counter, &pu16Rpm);
					if(pu16Rpm > (HOVER_ENGINE_CRUISE_RPM - HOVER_ENGINE_RPM_TOLERANCE)) // should be below "nominal rpm". are these the correct vaules?
						u8status = 0U;
			}

			// if the pod speed is lower than the pod standby speed, the pod is lifted, and all engines have "nominal rpm"
			if((u32PodSpeed < PODSPEED_STANDBY) && (sFCU.sOpStates.u8Lifted != 0U) && u16Rpm == 1){
				//Here we need to check which command was received
				
				//set gimbal angle to  0.5° for Static Gimbaling command
				//set gimbal angle to 2° for Full Backwards Gimbal command and set phase to Ready For Push
			}

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

