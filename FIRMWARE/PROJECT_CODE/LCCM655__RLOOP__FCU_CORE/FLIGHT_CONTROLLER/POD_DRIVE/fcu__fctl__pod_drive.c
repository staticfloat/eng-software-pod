/**
* @file       FCU__FCTRL__POD_DRIVE.C
* @brief      Pod drive control
* @author
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

// TODO: need the following constants and variables implemented:
#define POD_DRIVE_GS_COMM_LOSS_DELAY	10U	//TODO: find this value
#define POD_DRIVE_PRERUN_MAX_DELAY		10U //TODO: find this value
#define POD_TARGET_POINT_MARGIN_POS		10U //TODO: find this value
#define POD_STOP_X_POS					10U //TODO: find this value
#define PODSPEED_STANDBY				10U //BORROWED FROM HOVER ENGINE CONTROL
#define PODSPEED_MAX_SPEED_TO_STABILIZE 10U //TODO: find this value
#define PODSPEED_TOO_HIGH				10U
//add POD_STOP to E_GS_COMMANDS
Luint32 gs_comm_loss_timer;					//TODO: find where this is defined
Luint32 pre_run_state_timer;

// TODO: need the following functions implemented:
//vFCU_FLIGHTCTL_HOVERENGINES__disable()
//vFCU_FLIGHTCTL_HOVERENGINES__stop()
//vFCU_FLIGHTCTL_HOVERENGINES__enable()
//vFCU_FLIGHTCTL_HOVERENGINES__start()
//vFCU_FLIGHTCTL_AUX_PROP__stop()
//vFCU_FLIGHTCTL_AUX_PROP__disable()
//vFCU_FLIGHTCTL_GIMBAL__backward_drive_level()
//vFCU_FLIGHTCTL_GIMBAL__maintain_against_pusher_level()
//vFCU_FLIGHTCTL_GIMBAL__horizontal_level()
//vFCU_FLIGHTCTL_GIMBAL__forward_drive_level()
//vFCU_FLIGHTCTL_EDDY_BRAKES__release()
//vFCU_FLIGHTCTL_EDDY_BRAKES__disable()
//vFCU_FLIGHTCTL_XXXXXX__get_front_pod_pos()
//vFCU__POD_SPEED()	// BORROWED FROM HOVER ENGINE CONTROL
//vFCU_FLIGHTCTL_GET_GS_CMD()



void vFCU_FLIGHTCTL_PODDRIVE__stop(void)
{
	vFCU_FLIGHTCTL_HOVERENGINES__stop();
	vFCU_FLIGHTCTL_HOVERENGINES__disable();
	vFCU_FLIGHTCTL_AUX_PROP__stop();
	vFCU_FLIGHTCTL_AUX_PROP__disable();
}


void vFCU_FLIGHTCTL_PODDRIVE__Process(void)
{

	switch(sFCU.eMissionPhase)
	{
		case MISSION_PHASE__TEST_PHASE:
			if (gs_comm_loss_timer > POD_DRIVE_GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__stop();
			}
			break;

		case MISSION_PHASE__PRE_RUN_PHASE:
			if (gs_comm_loss_timer > POD_DRIVE_GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__stop();
			}
			else
			{
				vFCU_FLIGHTCTL_EDDY_BRAKES__release();
				vFCU_FLIGHTCTL_EDDY_BRAKES__disable();
				switch(sFCU.sPodDrive.ePreRunState)
				{
					// TODO: how to handle timeout for this state machine
					// who initializes this timer?  and do we go to the next state if timeout?
					// or go back to  init state? In addition to reporting error to GS
					case PODDRIVE_PRERUN_INIT:
						if (if(sFCU.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY)))
						{
							sFCU.sPodDrive.ePreRunState = PODDRIVE_PRERUN_START_HOVER_ENGINES;
						}
						break;
					case PODDRIVE_PRERUN_START_HOVER_ENGINES:
						vFCU_FLIGHTCTL_HOVERENGINES__enable();
						vFCU_FLIGHTCTL_HOVERENGINES__start();
						sFCU.sPodDrive.ePreRunState = PODDRIVE_PRERUN_PREPARE_FOR_PUSH;
						break;
					case PODDRIVE_PRERUN_PREPARE_FOR_PUSH:
						vFCU_FLIGHTCTL_GIMBAL__backward_drive_level();
						// not done here yet
						sFCU.sPodDrive.ePreRunState = PODDRIVE_PRERUN_WAIT_FOR_PUSH;
						break;
					case PODDRIVE_PRERUN_WAIT_FOR_PUSH:
						vFCU_FLIGHTCTL_GIMBAL__maintain_against_pusher_level();
						// not done here yet
						break;
				}

			}
			break;

		case MISSION_PHASE__PUSH_INTERLOCK_PHASE:
			if (gs_comm_loss_timer > POD_DRIVE_GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__stop();
			}
			else
			{
				vFCU_FLIGHTCTL_EDDY_BRAKES__release();
				vFCU_FLIGHTCTL_EDDY_BRAKES__disable();
				vFCU_FLIGHTCTL_GIMBAL__horizontal_level();
			}
			break;


		case MISSION_PHASE__FLIGHT_MODE:
			Luint32 u32PodPos = vFCU_FLIGHTCTL_XXXXXX__get_front_pod_pos();
			Luint32 u32PodSpeed = vFCU__POD_SPEED();
			E_GS_COMMANDS gs_cmd = vFCU_FLIGHTCTL_GET_GS_CMD();
			if (gs_cmd == POD_STOP) || (u32PodSpeed > PODSPEED_TOO_HIGH) ||
					(FCU watchdog indicates a failure) || (BMS failure require emergency breaking))
			{
				// controlled emergency breaking
				// add full eddy brakes until speed < PODSPEED_STANDBY
			}
			else
			{
				if ((POD_STOP_X_POS - u32PodPos) > POD_TARGET_POINT_MARGIN_POS)
				{
					// add control code:
					// eddy brakes direction and speed commands for pre-defined speed profile
				}
				if (u32PodSpeed > PODSPEED_MAX_SPEED_TO_STABILIZE)
				{
					vFCU_FLIGHTCTL_GIMBAL__horizontal_level();
				}
				else
				{
					// add control code:
					// gimballing HE's for pre-defined speed profile
				}
			}


			break;

		case RUN_STATE__FLIGHT_ABORT:	//TODO: verify this is post_run_phase
			if (gs_comm_loss_timer > POD_DRIVE_GS_COMM_LOSS_DELAY)
			{
				vFCU_FLIGHTCTL_PODDRIVE__stop();
			}
			else
			{
				Luint32 u32PodPos = vFCU_FLIGHTCTL_XXXXXX__get_front_pod_pos();
				Luint32 u32PodSpeed = vFCU__POD_SPEED();
				Luint8 u8Counter;

				if ((POD_STOP_X_POS - u32PodPos) > POD_TARGET_POINT_MARGIN_POS)
				{
					vFCU_FLIGHTCTL_GIMBAL__forward_drive_level();
				}
				else if (u32PodSpeed < PODSPEED_STANDBY)
				{
					// maybe this can be a function in the hover engine control
					vFCU_FLIGHTCTL_GIMBAL__horizontal_level();
					for(u8Counter = 1; u8Counter < 8; u8Counter++)
					{
						vFCU_THROTTLE__Set_Throttle(u8Counter, 0, THROTTLE_TYPE__STEP);
						// Is this needed? vFCU_COOLING__Set_Valve(u8Counter, 0.0, 2.0);
					}
					vFCU_FLIGHTCTL_EDDY_BRAKES__release();
					vFCU_FLIGHTCTL_EDDY_BRAKES__disable();
				}
			}
			break;
	}
}


#endif //C_LOCALDEF__LCCM655__ENABLE_HOVERENGINES_CONTROL
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

