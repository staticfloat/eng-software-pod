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

// add to fcu_core_enums.h in lift mech section:
typedef enum
{
	LIFTMECH_DIR_DOWN = 0U,
	LIFTMECH_DIR_UP = 1U

}E_FCU__LIFTMECH_DIRECTION;

void vFCU_FLIGHTCTL_LIFTMECH__Release(void)
{
	if ((sFCU.eMissionPhase == MISSION_PHASE__TEST_PHASE) || (sFCU.eMissionPhase == MISSION_PHASE__POST_RUN))
	{
		Luint32 u32PodSpeed = vFCU__POD_SPEED();
		if (sFCU.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY))
		{
			vFCU_FLIGHTCTL_LIFTMECH_Dir(LIFTMECH_DIR_UP);	//TODO: assuming 1 is up
			vFCU_FLIGHTCTL_LIFTMECH_Speed(C_FCU__LIFTMECH_ACTUATOR_NOM_UNLIFT_SPEED);	// TODO: until unlifted state is true
		}
	}
}

void vFCU_FLIGHTCTL_LIFTMECH__SetDirAll(E_FCU__LIFTMECH_DIRECTION dir)
{
	Luint8 u8actuatorIndex;

	for (u8actuatorIndex=0; u8actuatorIndex < NUM_LIFTMECH_ACTUATORS; u8actuatorIndex++)
	{
		vFCU_FLIGHTCTL_LIFTMECH_Dir(u8actuatorIndex, dir);
	}
}

void vFCU_FLIGHTCTL_LIFTMECH__SetSpeedAll(Luint32 u32speed)
{
	Luint8 u8actuatorIndex;

	for (u8actuatorIndex=0; u8actuatorIndex < NUM_LIFTMECH_ACTUATORS; u8actuatorIndex++)
	{
		vFCU_FLIGHTCTL_LIFTMECH_Speed(u8actuatorIndex, u32speed);
	}
}

void vFCU_FLIGHTCTL_LIFTMECH_Dir(Luint8 u8ActuatorIndex, E_FCU__LIFTMECH_DIRECTION dir)
{
	// need to interface with lower level stuff here
}

void vFCU_FLIGHTCTL_LIFTMECH_Speed(Luint8 u8ActuatorIndex, Luint32 u32speed)
{
	// need to interface with lower level stuff here
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
