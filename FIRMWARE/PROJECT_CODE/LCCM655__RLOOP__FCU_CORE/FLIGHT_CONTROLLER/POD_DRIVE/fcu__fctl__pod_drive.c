/**
* @file       FCU__FCTRL__POD_DRIVE.C
* @brief      Pod drive control
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
 * @addtogroup FCU__FLIGHT_CTL__PODDRIVE
 * @ingroup FCU
 * @{ */


#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_PODDRIVE_CONTROL == 1U

extern struct _strFCU sFCU;

// TODO: need values for following constants and move to fcu_core_defines.h
#define C_FCU__GS_COMM_LOSS_DELAY	// should go in udp rx section
#define C_FCU__POD_TARGET_POINT_MARGIN_POS
#define C_FCU__POD_STOP_X_POS
#define C_FCU__PODSPEED_STANDBY
#define C_FCU__PODSPEED_MAX_SPEED_TO_STABILIZE
#define C_FCU__PODSPEED_TOO_HIGH
#define C_FCU__LIFTMECH_RETRACTED_MLP_DISTANCE
#define C_FCU__LIFTMECH_ACTUATOR_NOM_UNLIFT_SPEED

// TODO: add variable to fcu_core.h inside of sUDPDiag
// implement like sFCU.sPodDrive.u8100MS_Timer (reset after heartbeat is received?, heartbeat to be implemented)
// need 10MS ISR func for udp to increment this
Luint32 u32_10MS_GS_COMM_Timer;
// add vFCU_NET_RX__GetGsCommTimer() to fcu_core_net_rx.c

// TODO: add struct to fcu_core.h inside of _strFCU:
#if C_LOCALDEF__LCCM655__ENABLE_PODDRIVE_CONTROL == 1U
struct
{
	E_FCU__PODDRIVE_PRERUN_STATE ePreRunState;
	E_FCU__PODDRIVE_GS_COMM eGSCommand;
	Luint8 u8100MS_Timer;

}sPodDrive;
#endif

// add to fcu_core_enums.h in pod drive section:
typedef enum
{
	PODDRIVE_GS_POD_STOP = 0U
}E_FCU__PODDRIVE_GS_COMM;

typedef enum
{
	PODDRIVE_PRERUN_INITIAL_STATE = 0U,
	PODDRIVE_PRERUN_START_HE_STATE = 1U,
	PODDRIVE_PRERUN_RETRACT_LIFTMECH = 2U,
	PODDRIVE_PRERUN_GIMBAL_BACKWARD = 3U,
	PODDRIVE_PRERUN_GIMBAL_MAINTAIN = 4U
}E_FCU__PODDRIVE_PRERUN_STATE;

// add to fcu_core_enums.h in pod drive section:
typedef enum
{
	GIMBAL_NEUTRAL_LEVEL = 0U,
	GIMBAL_BACKWARD_LEVEL = 1U,
	GIMBAL_FORWARD_LEVEL = 2U
}E_FCU__GIMBAL_LEVEL;



// TODO: need the following functions:
//vFCU_FLIGHTCTL_HOVERENGINES__stop()
//vFCU_FLIGHTCTL_HOVERENGINES__start()
//vFCU_FLIGHTCTL_AUX_PROP__Stop()
//vFCU_FLIGHTCTL_AUX_PROP__Disable()
//vFCU_FLIGHTCTL_GIMBAL__SetLevel(GIMBAL_BACKWARD_LEVEL/GIMBAL_NEUTRAL_LEVEL/GIMBAL_FORWARD_LEVEL)
//vFCU_FLIGHTCTL_EDDY_BRAKES__Release()
//vFCU_FLIGHTCTL_EDDY_BRAKES__GainScheduleController(speed)	// pid controller for brakes until position < C_FCU__POD_STOP_X_POS
//vFCU_FLIGHTCTL_EDDY_BRAKES__GimbalSpeedController() // pid controller for gimbals when speed < C_FCU__PODSPEED_MAX_SPEED_TO_STABILIZE
//vFCU_FLIGHTCTL_EDDY_BRAKES__ApplyFullBrakes()  // add full eddy brakes until speed < C_FCU__PODSPEED_STANDBY
//vFCU_FLIGHTCTL_XXXXXX__GetFrontPos()
//vFCU_FLIGHTCTL_LIFTMECH__Dir(index,dir)
//vFCU_FLIGHTCTL_LIFTMECH__Speed(index, speed)
//vFCU_FLIGHTCTL_LIFTMECH__Get_MLP()
//vFCU_PUSHER__GetState()
//vFCU__POD_SPEED()	// BORROWED FROM HOVER ENGINE CONTROL
//vFCU_NET_RX__GetGsCommTimer()



void vFCU_FLIGHTCTL_PODDRIVE__Stop(void)
{
	vFCU_FLIGHTCTL_HOVERENGINES__stop();
	vFCU_FLIGHTCTL_AUX_PROP__Stop();
	vFCU_FLIGHTCTL_AUX_PROP__Disable();
}


void vFCU_FLIGHTCTL_PODDRIVE__Process(void)
{

	switch(sFCU.eMissionPhase)
	{
		Luint32 u32PodSpeed;
		Luint32 u32PodPos;

		case MISSION_PHASE__TEST_PHASE:
			if (vFCU_NET_RX__GetGsCommTimer() > C_FCU__GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__Stop();
			}
			break;

		case MISSION_PHASE__PRE_RUN_PHASE:
			Luint8 u8LiftMechIndex;

			if (vFCU_NET_RX__GetGsCommTimer() > C_FCU__GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__Stop();
			}
			else
			{
				switch(sFCU.sPodDrive.ePreRunState)
				case PODDRIVE_PRERUN_INITIAL_STATE:
					if (sFCU.sPodDrive.u8100MS_Timer == 0)
					{
						vFCU_FLIGHTCTL_EDDY_BRAKES__Release();
					}
					u32PodSpeed = vFCU__POD_SPEED();
					if (sFCU.sOpStates.u8Lifted && (u32PodSpeed < C_FCU__PODSPEED_STANDBY)))
					{
						sFCU.sPodDrive.ePreRunState = PODDRIVE_PRERUN_START_HE_STATE;
						sFCU.sPodDrive.u8100MS_Timer = 0;
					}
					if (sFCU.sPodDrive.u8100MS_Timer >= 100)
					{
						// TODO: report error to ground station
						// exit?
					}
					break;
				case PODDRIVE_PRERUN_START_HE_STATE:
					if (sFCU.sPodDrive.u8100MS_Timer == 0)
					{
						vFCU_FLIGHTCTL_HOVERENGINES__start();
					}
					if (vFCU_FLIGHTCTL_HOVERENGINES__Get_State() == HOVERENGINES_STATE__STATIC_HOVERING)
					{
						sFCU.sPodDrive.ePreRunState = PODDRIVE_PRERUN_RETRACT_LIFTMECH;
						sFCU.sPodDrive.u8100MS_Timer = 0;
					}
					if (sFCU.sPodDrive.u8100MS_Timer >= 100)
					{
						// TODO: report error to ground station
						// exit?
					}
					break;
				case PODDRIVE_PRERUN_RETRACT_LIFTMECH:
					if (sFCU.sPodDrive.u8100MS_Timer == 0)
					{
						for (u8LiftMechIndex=0; u8LiftMechIndex < NUM_LIFTMECH_ACTUATORS; u8LiftMechIndex++)
						{
							vFCU_FLIGHTCTL_LIFTMECH_Dir(u8LiftMechIndex, 1);	//TODO: assuming 1 is up
							vFCU_FLIGHTCTL_LIFTMECH_Speed(u8LiftMechIndex, C_FCU__LIFTMECH_ACTUATOR_NOM_UNLIFT_SPEED);
						}
					}
					if (vFCU_FLIGHTCTL_LIFTMECH__Get_MLP() < C_FCU__LIFTMECH_RETRACTED_MLP_DISTANCE)
					{
						sFCU.sPodDrive.ePreRunState = PODDRIVE_PRERUN_GIMBAL_BACKWARD;
						sFCU.sPodDrive.u8100MS_Timer = 0;
					}
					if (sFCU.sPodDrive.u8100MS_Timer >= 100)
					{
						// TODO: report error to ground station
						// exit?
					}
					break;
				case PODDRIVE_PRERUN_GIMBAL_BACKWARD:
					if (sFCU.sPodDrive.u8100MS_Timer == 0)
					{
						vFCU_GIMBAL__SetLevel(GIMBAL_BACKWARD_LEVEL);
					}
					if (vFCU_PUSHER__GetState() == 1U)	// connected to pusher
					{
						sFCU.sPodDrive.ePreRunState = PODDRIVE_PRERUN_GIMBAL_MAINTAIN;
						sFCU.sPodDrive.u8100MS_Timer = 0;
					}
					if (sFCU.sPodDrive.u8100MS_Timer >= 100)
					{
						// TODO: report error to ground station
						// exit?
					}
					break;
				case PODDRIVE_PRERUN_GIMBAL_MAINTAIN:
					// maintain gimbal: do nothing, we are already gimbaling backward
					break;
			}
			break;

		case MISSION_PHASE__PUSH_INTERLOCK_PHASE:
			if (vFCU_NET_RX__GetGsCommTimer() > C_FCU__GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__Stop();
			}
			else
			{
				vFCU_FLIGHTCTL_EDDY_BRAKES__Release();
				vFCU_FLIGHTCTL_GIMBAL__SetLevel(GIMBAL_NEUTRAL_LEVEL);
			}
			break;


		case MISSION_PHASE__FLIGHT_MODE:
			u32PodPos = vFCU_FLIGHTCTL_XXXXXX__GetFrontPos();
			u32PodSpeed = vFCU__POD_SPEED();

			if (sFCU.sPodDrive.eGSCommand == PODDRIVE_GS_POD_STOP;) || (u32PodSpeed > C_FCU__PODSPEED_TOO_HIGH) || (FCU watchdog failure))
			{
				// controlled emergency breaking
				vFCU_FLIGHTCTL_EDDY_BRAKES__ApplyFullBrakes();  // add full eddy brakes until speed < C_FCU__PODSPEED_STANDBY
			}
			else
			{
				vFCU_FLIGHTCTL_EDDY_BRAKES__GainScheduleController(u32PodSpeed);	// pid controller for brakes until position < C_FCU__POD_STOP_X_POS
				if (u32PodSpeed >= C_FCU__PODSPEED_MAX_SPEED_TO_STABILIZE)
				{
					vFCU_FLIGHTCTL_GIMBAL__SetLevel(GIMBAL_NEUTRAL_LEVEL);
				}
				else
				{
					vFCU_FLIGHTCTL_EDDY_BRAKES__GimbalSpeedController();	// pid controller for gimbals when speed < C_FCU__PODSPEED_MAX_SPEED_TO_STABILIZE
				}
			}
			break;

		case MISSION_PHASE__POST_RUN:
			// TODO: need to rethink this part if hover engines are not running in this phase
			if (vFCU_NET_RX__GetGsCommTimer() > C_FCU__GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__Stop();
			}
			else
			{
				Luint32 u32PodPos = vFCU_FLIGHTCTL_XXXXXX__GetFrontPos();
				Luint32 u32PodSpeed = vFCU__POD_SPEED();

				vFCU_FLIGHTCTL_EDDY_BRAKES__ApplyFullBrakes();  // add full eddy brakes until speed < C_FCU__PODSPEED_STANDBY; keep applying if called again
				if ((C_FCU__POD_STOP_X_POS - u32PodPos) > C_FCU__POD_TARGET_POINT_MARGIN_POS)
				{
					vFCU_FLIGHTCTL_GIMBAL__SetLevel(GIMBAL_FORWARD_LEVEL);	// until reach target position
					vFCU_FLIGHTCTL_EDDY_BRAKES__Release();
				}
				else if (u32PodSpeed < C_FCU__PODSPEED_STANDBY)
				{
					vFCU_FLIGHTCTL_GIMBAL__SetLevel(GIMBAL_NEUTRAL_LEVEL);
					vFCU_FLIGHTCTL_HOVERENGINES__stop()
				}
			}
			break;
	}
}


void vFCU_FLIGHTCTL_PODDRIVE__SetPodStopCmd(void)
{
	sFCU.sPodDrive.eGSCommand = PODDRIVE_GS_POD_STOP;
}

void vFCU_FLIGHTCTL_PODDRIVE__100MS_ISR(void)
{
	sFCU.sPodDrive.u8100MS_Timer++;
}



#endif //C_LOCALDEF__LCCM655__ENABLE_PODDRIVE_CONTROL
#ifndef C_LOCALDEF__LCCM655__ENABLE_PODDRIVE_CONTROL
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

