/**
 * @file		FCU__FCTL__MAIN_SM.C
 * @brief		Main state machine for the flight control unit
 * @author		Lachlan Grogan, Marek Gutt-Mostowy
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
 * @addtogroup FCU__FCTL__MAIN_SM
 * @ingroup FCU
 * @{ */

#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U

//the structure
extern struct _strFCU sFCU;

//needs to be defined in the DB
#define POD_MIN_X_POS 				500U


//TODO: need the following function implemented

//vFCU_FLIGHTCTRL_XXXXXX__GET_POD_SPEED();
//vFCU_FLIGHTCTL_XXXXXX__GET_POD_REAR_X_POS();
//vFCU_FLIGHTCTL_XXXXXX__GET_POD_VEL();

/***************************************************************************//**
 * @brief
 * Init any variables as is needed by the main state machine
 * 
 * @st_funcMD5		5224534176289DBD9FF1B39936308C7E
 * @st_funcID		LCCM655R0.FILE.020.FUNC.001
 */
void vFCU_FCTL_MAINSM__Init(void)
{
	sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__RESET;

	//init the auto sequence
	vFCU_MAINSM_AUTO__Init();

}


/***************************************************************************//**
 * @brief
 * Process the main state machine
 * 
 * @st_funcMD5		2C23D1564E9845C3BED5E00B06C0BBB3
 * @st_funcID		LCCM655R0.FILE.020.FUNC.002
 */
void vFCU_FCTL_MAINSM__Process(void)
{
	Luint8 u8Counter;
	Luint8 u8Test;
	Luint8 u8TestsSuccesful;

	//handle the state machine.
	switch(sFCU.sStateMachine.eMissionPhase)
	{

		case MISSION_PHASE__RESET:
			//we have just come out of reset here.
			//init our rPod specific systems

			//pusher
			#if C_LOCALDEF__LCCM655__ENABLE_PUSHER == 1U
				vFCU_PUSHER__Init();
			#endif

			//laser contrast sensors
			#if C_LOCALDEF__LCCM655__ENABLE_LASER_CONTRAST == 1U
				vFCU_LASERCONT__Init();
			#endif

			//laser distance
			#if C_LOCALDEF__LCCM655__ENABLE_LASER_DISTANCE == 1U
				vFCU_LASERDIST__Init();
			#endif

			//init the brake systems
			#if C_LOCALDEF__LCCM655__ENABLE_BRAKES == 1U
				vFCU_BRAKES__Init();
			#endif

			//Init the throttles
			#if C_LOCALDEF__LCCM655__ENABLE_THROTTLE == 1U
				vFCU_THROTTLE__Init();
			#endif

			//init the ASI RS485 interface
			#if C_LOCALDEF__LCCM655__ENABLE_ASI_RS485 == 1U
				vFCU_ASI__Init();
			#endif

			//init the acclerometer system
			#if C_LOCALDEF__LCCM655__ENABLE_ACCEL == 1U
				vFCU_ACCEL__Init();
			#endif

			//laser opto's
			#if C_LOCALDEF__LCCM655__ENABLE_LASER_OPTONCDT == 1U
				vFCU_LASEROPTO__Init();
			#endif

			//PiComms Layer
			#if C_LOCALDEF__LCCM655__ENABLE_PI_COMMS == 1U
				vFCU_PICOMMS__Init();
			#endif

			//finally init the flight controller
			#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
				vFCU_FLIGHTCTL__Init();
			#endif

			//start the LGU interface
			#if C_LOCALDEF__LCCM655__LGU_COMMS_SYSTEM == 1U
				vFCU_LGU__Init();
			#endif

			//put the flight computer into startup mode now that everything has been initted.
			sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__TEST;

			break;


		case MISSION_PHASE__TEST:
			
			//in this mode we are performing tests of all of the sensors and motors pl an auto-sequence

			//see if we have an auto sequence abort
			u8Test = u8FCU_MAINSM_AUTO__Is_Abort();
			if(u8Test == 1U)
			{
				//move to RESET if tests aborted
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__RESET;
			}
			else
			{
				u8Test = u8FCU_MAINSM_AUTO__Is_Busy();
				if(u8Test == 1U)
				{
					//stay in state, we are still busy
				}
				else
				{
					//not busy and not abort, set flag indicating tests succesful
					u8TestsSuccesful = 1U;
				}
			}

			//Get Pod Speed
			Luint32 u32PodSpeed = vFCU_FLIGHTCTRL_XXXXXX__GET_POD_SPEED();

			if (u8TestsSuccesful == 1U && sFCU.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY))
			{

					sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__PRE_RUN;
			}
			else
			{

				if(u32PodSpeed < PODSPEED_STANDBY)
				{
					sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__TEST;
				}
				else
				{
					sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__PUSHER_INTERLOCK;
				}

			}


			break;


		case MISSION_PHASE__PRE_RUN:

			//Transition to Pusher Interlock Phase based on the acceleration
			//Not sure about how to call the right accelerometer and whether it indicates the right axis
			if(sFCU.sAccel.sChannels[C_LOCALDEF__LCCM418__NUM_DEVICES].s32CurrentAccel_mmss > PODSPEED_STANDBY) 
			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__PUSHER_INTERLOCK;
			}
			else
			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__PRE_RUN;
			}

			break;


		case MISSION_PHASE__PUSHER_INTERLOCK:

			//transition to FLIGHT mode if the pod has reached the min_x_pos AND 1 second elapsed from the disconnection from the pusher
			
			Luint8   PUSHER_STATE = u8FCU_PUSHER__Get_PusherState(void);
			Luint32  POD_REAR_X_POS = vFCU_FLIGHTCTL_XXXXXX__GET_POD_REAR_X_POS();


			if(PUSHER_STATE == 0U)
			{
				//Enable the counter
				sFCU.sStateMachine.EnableCounter == 1U;
			}
			else
			{
				//do nothing
			}

			if((POD_REAR_X_POS > POD_MIN_X_POS) && (sFCU.sStateMachine.Counter == 10U))
			{
				//Switch to Mission Phase Flight
				sFCU.eMissionPhase = MISSION_PHASE__FLIGHT;
				//Disable the counter
				sFCU.sStateMachine.EnableCounter = 0U;
				//Reset the counter
				sFCU.sStateMachine.Counter = 0U;

			}
			else
			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__PUSHER_INTERLOCK;
			}

			break;


		case MISSION_PHASE__FLIGHT:
			//this is the flight mode controller

			#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
				vFCU_FLIGHTCTL__Process();
			#endif

			Luint32 u32PodVel = vFCU_FLIGHTCTL_XXXXXX__GET_POD_VEL();
			
			if(u32PodVel < PODSPEED_STANDBY)

			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__POST_RUN;
			}

			else
			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__FLIGHT;
			}

			break;

		case MISSION_PHASE__POST_RUN:
			//post run state

			Luint32 u32PodVel = vFCU_FLIGHTCTL_XXXXXX__GET_POD_VEL();
			void vFCU_FCTL_MAINSM__EnterPreRun_Phase(Luint32 u32Key)

			if(u32PodVel < PODSPEED_STANDBY) && (sFCU.sStateMachine.eGSCommands == POST_RUN_TO_PRE_RUN)
			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__PRE_RUN;
			}
			else
			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__POST_RUN;
			}
			break;

		default:
			//should not get here
			break;

	}//switch(sFCU.eRunState)

	//always process these items after we have been initted
	if(sFCU.sStateMachine.eMissionPhase > MISSION_PHASE__RESET)
	{

		//process the SC16IS interface always
		#if C_LOCALDEF__LCCM487__ENABLE_THIS_MODULE == 1U
			for(u8Counter = 0U; u8Counter < C_LOCALDEF__LCCM487__NUM_DEVICES; u8Counter++)
			{
				vSC16__Process(u8Counter);
			}
		#endif

		// process the AMC7812
		// process the throttles
		#if C_LOCALDEF__LCCM655__ENABLE_THROTTLE == 1U
			vFCU_THROTTLE__Process();
			vAMC7812__Process();
		#endif

		#if C_LOCALDEF__LCCM655__ENABLE_LASER_OPTONCDT == 1U
			vFCU_LASEROPTO__Process();
		#endif

		//laser orientation
		#if C_LOCALDEF__LCCM655__ENABLE_ORIENTATION == 1U
			vFCU_LASER_ORIENTATION__Process();
		#endif

		#if C_LOCALDEF__LCCM655__ENABLE_LASER_CONTRAST == 1U
			vFCU_LASERCONT__Process();
		#endif

		#if C_LOCALDEF__LCCM655__ENABLE_LASER_DISTANCE == 1U
			vFCU_LASERDIST__Process();
		#endif

		#if C_LOCALDEF__LCCM655__ENABLE_PUSHER == 1U
			vFCU_PUSHER__Process();
		#endif

		//process the brakes.
		#if C_LOCALDEF__LCCM655__ENABLE_BRAKES == 1U
			vFCU_BRAKES__Process();
		#endif

		//process the accel channels
		#if C_LOCALDEF__LCCM655__ENABLE_ACCEL == 1U
			vFCU_ACCEL__Process();
		#endif

		//ASI RS485 interface
		#if C_LOCALDEF__LCCM655__ENABLE_ASI_RS485 == 1U
			vFCU_ASI__Process();
		#endif

		//process any Pi Comms
		#if C_LOCALDEF__LCCM655__ENABLE_PI_COMMS == 1U
			vFCU_PICOMMS__Process();
		#endif

		//start the LGU interface
		#if C_LOCALDEF__LCCM655__LGU_COMMS_SYSTEM == 1U
			vFCU_LGU__Process();
		#endif

		//process auto-sequence control
		vFCU_MAINSM_AUTO__Process();



	}
	else
	{
		//do nothing.
	}


}

//allows us to enter pre-run phase from ethernet
void vFCU_FCTL_MAINSM__EnterPreRun_Phase(Luint32 u32Key)
{

}

void vFCU_FCTL_MAINSM__100MS_ISR(void)
{
   if (sFCU.sStateMachine.EnableCounter == 1U)
   {
        sFCU.sStateMachine.Counter++;
   }
}

#endif //#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
//safetys
#ifndef C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE
	#error
#endif
/** @} */
/** @} */
/** @} */

