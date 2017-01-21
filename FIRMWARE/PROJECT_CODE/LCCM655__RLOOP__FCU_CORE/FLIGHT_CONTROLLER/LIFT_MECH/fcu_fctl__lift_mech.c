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

typedef enum
{
	LIFTMECH_AftLeft = 0U,
	LIFTMECH_AftRight = 1U,
	LIFTMECH_ForwardLeft = 2U,
	LIFTMECH_ForwardRight = 3U
}E_FCU__LIFTMECH_ACTUATOR;


// GS comm packet types to add to fcu_core_net_packet_types.h
NET_PKT__FCU_LIFTMECH__SET_DIR
NET_PKT__FCU_LIFTMECH__SET_SPEED
NET_PKT__FCU_LIFTMECH__SET_GROUP_DIR
NET_PKT__FCU_LIFTMECH__SET_GROUP_SPEED

// GS lift mech commands to add to fcu_core_net_rx.c:
// assuming get actuator and value in u32blocks
case NET_PKT__FCU_LIFTMECH__SET_DIR:
	//set direction of specific mech lift
	#if C_LOCALDEF__LCCM655__ENABLE_LIFT_MECH_CONTROL == 1U
		E_FCU__LIFTMECH_ACTUATOR actuator;
		E_FCU__LIFTMECH_DIRECTION dir;
		switch(u32Block[0])
		{
			case 0:
				actuator = LIFTMECH_AftLeft;
				break;
			case 1:
				actuator = LIFTMECH_AftRight;
				break;
			case 2:
				actuator = LIFTMECH_ForwardLeft;
				break;
			case 3:
				actuator = LIFTMECH_ForwardRight;
				break;
			case default:
				//report error
				break;
		}
		switch(u32Block[1])
		{
			case 0:
				dir = LIFTMECH_DIR_DOWN;
				break;
			case 1:
				dir = LIFTMECH_DIR_UP;
				break;
			case default:
				//report error
				break;
		}
		vFCU_FLIGHTCTL_LIFTMECH_Dir(actuator, dir);
	#endif
	break;

case NET_PKT__FCU_LIFTMECH__SET_SPEED:
	//set speed of specific mech lift
	#if C_LOCALDEF__LCCM655__ENABLE_LIFT_MECH_CONTROL == 1U
		E_FCU__LIFTMECH_ACTUATOR actuator;
		switch(u32Block[0])
		{
			case 0:
				actuator = LIFTMECH_AftLeft;
				break;
			case 1:
				actuator = LIFTMECH_AftRight;
				break;
			case 2:
				actuator = LIFTMECH_ForwardLeft;
				break;
			case 3:
				actuator = LIFTMECH_ForwardRight;
				break;
			case default:
				//report error
				break;
		}
		vFCU_FLIGHTCTL_LIFTMECH_Speed(actuator, u32Block[1]);
	#endif
	break;

case NET_PKT__FCU_LIFTMECH__SET_GROUP_DIR:
	//set direction of all mech lift actuators
	#if C_LOCALDEF__LCCM655__ENABLE_LIFT_MECH_CONTROL == 1U
		E_FCU__LIFTMECH_DIRECTION dir;
		switch(u32Block[0])
		{
			case 0:
				dir = LIFTMECH_DIR_DOWN;
				break;
			case 1:
				dir = LIFTMECH_DIR_UP;
				break;
			case default:
				//report error
				break;
		}
		vFCU_FLIGHTCTL_LIFTMECH__SetDirAll(dir);
	#endif
	break;

case NET_PKT__FCU_LIFTMECH__SET_GROUP_SPEED:
	//set speed of all mech lift actuators
	#if C_LOCALDEF__LCCM655__ENABLE_LIFT_MECH_CONTROL == 1U
		vFCU_FLIGHTCTL_LIFTMECH__SetSpeedAll(u32Block[0]);
	#endif
	break;


// start of this module
void vFCU_FLIGHTCTL_LIFTMECH__Release(void)
{
	// this seems similar to step 3 in 9.2, but that one is in prerun phase
	// and checks Get_MLP is C_FCU__LIFTMECH_RETRACTED_MLP_DISTANCE
	// is that the same as checking for unlifted?
	if ((sFCU.eMissionPhase == MISSION_PHASE__TEST_PHASE) || (sFCU.eMissionPhase == MISSION_PHASE__POST_RUN))
	{
		Luint32 u32PodSpeed = vFCU__POD_SPEED();
		if (sFCU.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY))
		{
			vFCU_FLIGHTCTL_LIFTMECH__SetDirAll(LIFTMECH_DIR_UP);
			// TODO: until unlifted state is true, not sure who sets unlifted state, and how to check it
			vFCU_FLIGHTCTL_LIFTMECH__SetSpeedAll(C_FCU__LIFTMECH_ACTUATOR_NOM_UNLIFT_SPEED);
		}
	}
}

void vFCU_FLIGHTCTL_LIFTMECH__Get_MLP(void)
{
	// report MLP distance
	// add lower level function for this
	// not sure how to combine 4 values into one
}

void vFCU_FLIGHTCTL_LIFTMECH__SetDirAll(E_FCU__LIFTMECH_DIRECTION dir)
{
	vFCU_FLIGHTCTL_LIFTMECH_Dir(LIFTMECH_AftLeft, dir);
	vFCU_FLIGHTCTL_LIFTMECH_Dir(LIFTMECH_AftRight, dir);
	vFCU_FLIGHTCTL_LIFTMECH_Dir(LIFTMECH_ForwardLeft, dir);
	vFCU_FLIGHTCTL_LIFTMECH_Dir(LIFTMECH_ForwardRight, dir);
}

void vFCU_FLIGHTCTL_LIFTMECH__SetSpeedAll(Luint32 u32speed)
{
	vFCU_FLIGHTCTL_LIFTMECH_Speed(LIFTMECH_AftLeft, u32speed);
	vFCU_FLIGHTCTL_LIFTMECH_Speed(LIFTMECH_AftRight, u32speed);
	vFCU_FLIGHTCTL_LIFTMECH_Speed(LIFTMECH_ForwardLeft, u32speed);
	vFCU_FLIGHTCTL_LIFTMECH_Speed(LIFTMECH_ForwardRight, u32speed);
}

void vFCU_FLIGHTCTL_LIFTMECH_Dir(E_FCU__LIFTMECH_ACTUATOR actuator, E_FCU__LIFTMECH_DIRECTION dir)
{
	// need to interface with lower level stuff here
}

void vFCU_FLIGHTCTL_LIFTMECH_Speed(E_FCU__LIFTMECH_ACTUATOR actuator, Luint32 u32speed)
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
