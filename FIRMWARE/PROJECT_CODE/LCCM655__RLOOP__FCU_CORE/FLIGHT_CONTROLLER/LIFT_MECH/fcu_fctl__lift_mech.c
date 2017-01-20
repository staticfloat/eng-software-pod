/**
* @file       FCU__FCTRL__LIFT_MECH.C
* @brief      Lift Mechanism Control
* @author	  Nazneen Khan
* @copyright  rLoop Inc.
*/
/**
 * @addtogroup RLOOP
 * @{ */
/**
 * @addtogroup FCU
 * @ingroup RLOOP
 * @{ */
/**
 * @addtogroup FCU__FLIGHT_CTL__LIFTMECH
 * @ingroup FCU
 * @{ */


#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_LIFT_MECH_CONTROL == 1U

extern struct _strFCU sFCU;


void vFCU_FLIGHTCTL_LIFTMECH__Release(void)
{
	if ((sFCU.eMissionPhase == MISSION_PHASE__TEST_PHASE) || (sFCU.eMissionPhase == MISSION_PHASE__POST_RUN))
	{
		Luint32 u32PodSpeed = vFCU__POD_SPEED();
		if (sFCU.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY))
		{
			vFCU_FLIGHTCTL_LIFTMECH_DIR(1);	//TODO: assuming 1 is up
			vFCU_FLIGHTCTL_LIFTMECH_SPEED(LIFTMECH_ACTUATOR_NOM_UNLIFT_SPEED);	// TODO: until unlifted state is true
		}
	}
}

#endif //C_LOCALDEF__LCCM655__ENABLE_LIFT_MECH_CONTROL
#ifndef C_LOCALDEF__LCCM655__ENABLE_LIFT_MECH_CONTROL
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
