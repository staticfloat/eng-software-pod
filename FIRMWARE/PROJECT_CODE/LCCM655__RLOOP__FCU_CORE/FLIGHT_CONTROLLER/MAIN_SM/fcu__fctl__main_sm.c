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
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FCTL_MAIN_SM == 1U

//the structure
extern struct _strFCU sFCU;

//TODO:
//needs to be defined in fcu_core__localdef.h under 

//TODO:
//needs to be defined in the DB
#define C_FCU__NAV_POD_MIN_X_POS					(500U)
#define C_FCU__NAV_MAX_UNLIFTED_HEIGHT				(10) //define exact value
#define C_FCU__MAIN_SM_PUSHER_RELEASE_DELAY			(10U)


//TODO: Process timer function needs to be called in fcu_core.c under vFCU__RTI_100MS_ISR(void)

//state machine timed processes
#if C_LOCALDEF__LCCM655__ENABLE_FCTL_MAIN_SM == 1U
	vFCU_FCTL_MAINSM__100MS_ISR();
#endif

//TODO: need the following function implemented

//u32FCU_FLIGHTCTL_NAV__PodSpeed();
//u32FCU_FLIGHTCTL_NAV__GetFrontPos();
//u32FCU_FLIGHTCTL_NAV__GetRearPos();
//u32FCU_FLIGHTCTL_NAV__PodSpeed();
//s16FCU_FLIGHTCTL_LASERORIENT__Get_Z_Pos()
//u32LandingGearMLPLeftAftValue = u32FCU_LGU__Get_MLP_Value(Luint8 u8Counter);

//TODO: Add struct to fcu_core.h inside of _strFCU:

/** State Machine Structure **/
struct
{
	/** The mission phases
	 * http://confluence.rloop.org/display/SD/1.+Determine+Mission+Phases+and+Operating+States
	 * */
	E_FCU__MISSION_PHASE_T eMissionPhase;

	/** Counter to count the time elapsed from the disconnection from the pusher **/
	Luint32 Counter;

	/** Enable Counter counting time elapsed from the disconnection from the pusher **/
	Luint8 EnableCounter;

	/** Enum for Pod Status for SpaceX telemetry */
	E_FCU__POD_STATUS ePodStatus;

	/** Enum for GS commands */
	E_FCU__MAINSM_GS_COMM eGSCommand;

	/** Operating States Structure*/
	struct
	{
		/** Lifted State */
		Luint8 u8Lifted;

		/** Unlifted State */
		Luint8 u8Unlifted;

		/** Static Hovering */
		Luint8 u8StaticHovering;

		/** Gimballing Adjustment State */
		Luint8 u8GimbAdj;

		/** Ready For Push State */
		Luint8 u8ReadyForPush;

		/** Pushing State */
		Luint8 u8Pushing;

		/** Coast State */
		Luint8 u8Coast;

		/** Braking State */
		Luint8 u8Braking;

		/** Controlled Emergency State */
		Luint8 u8CtlEmergBraking;

	}sOpStates

}sStateMachine

//TODO: Add to fcu_core__types.h

/** Mission Phase Types */
typedef enum
{
	/** Come out of reset and handle any startup tasks. This is done when
	 * power is first applied to the FCU*/
	MISSION_PHASE__RESET = 0U,

	/** Run the flight computer in startup, do any diagnostics, etc
	 * Diagnostics here will be on systems that do not involve actuators such as
	 * memory CRC tests, etc.
	 * We can stay in startup mode, or startup-fail mode if something is not right here.*/
	MISSION_PHASE__TEST,

	/** In this mode the pod takes care of its functional tests as a terminal countdown.
	 * Autosequence is entered by the GS and once Autosequence tests are complete we
	 * move to flight mode. */
	MISSION_PHASE__PRE_RUN,

	MISSION_PHASE__PUSHER_INTERLOCK,

	/** Run the flight computer in flight mode, the flight controller takes care
	 * of everything until flight finished*/
	MISSION_PHASE__FLIGHT,

	/** we have aborted flight, need to cleanup systems, landing gear and safe the pod.
	 * This mode can also be the flight finished mode. */
	MISSION_PHASE__POST_RUN,

}E_FCU__MISSION_PHASE_T;

//TODO: Add to fcu_core__enums.h

typedef enum
{
	/** If tests failed */
	POD_STATUS__FAULT = 0U,

	/** Otherwise */
	POD_STATUS__IDLE = 1U,

	/** If ready for push */
	POD_STATUS__READY = 2U,

	/** If pushing */
	POD_STATUS__PUSHING = 3U,

	/** If coasting */
	POD_STATUS__COAST = 4U,

	/** If braking */
	POD_STATUS__BRAKING = 5U

}E_FCU__POD_STATUS;

typedef enum
{
	MAINSM_GS_NO_CMD = 0U
	MAINSM_GS_ENTER_PRE_RUN_PHASE = 1U
}E_FCU__MAINSM_GS_COMM;

//TODO: GS transition to PRE RUN PHASE command to add to fcu_core_net_rx.c:
case NET_PKT__FCU_GEN__ENTER_PRE_RUN_PHASE_COMMAND:
	#if C_LOCALDEF__LCCM655__ENABLE_MAINSM_CONTROL == 1U
		vFCU_FCTL_MAINSM__EnterPreRun_Phase();
	#endif
	break;


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
	Lint32 s32Accelmmss;
	Luint32 u32PodSpeed;
	Luint8 u8PusherState;

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
			Luint32 u32PodSpeed = u32FCU_FLIGHTCTL_NAV__PodSpeed();

			if (u8TestsSuccesful == 1U && sFCU.sStateMachine.sOpStates.u8Lifted && (u32PodSpeed < PODSPEED_STANDBY))
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
			s32Accelmmss = s32FCU_FCTL_NAV__Get_Accel_mmss();

			if(s32Accelmmss > PODSPEED_STANDBY) 
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
			
			u8PusherState = u8FCU_PUSHER__Get_PusherState(void);
			u32PodRearPos = u32FCU_FLIGHTCTL_NAV__GetRearPos();


			if(u8PusherState == 0U)
			{
				//Enable the counter
				sFCU.sStateMachine.EnableCounter == 1U;
			}
			else
			{
				//do nothing
			}

			if((u32PodRearPos > POD_MIN_X_POS) && (sFCU.sStateMachine.Counter >= PUSHER_RELEASE_DELAY))
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

			u32PodSpeed = u32FCU_FLIGHTCTL_NAV__PodSpeed();
			
			if(u32PodSpeed < PODSPEED_STANDBY)

			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__POST_RUN;
			}

			else
			{
				sFCU.sStateMachine.eMissionPhase = MISSION_PHASE__FLIGHT;
			}

			//Disable the counter
			sFCU.sStateMachine.EnableCounter == 0U;

			break;

		case MISSION_PHASE__POST_RUN:
			//post run state

			u32PodSpeed = u32FCU_FLIGHTCTL_NAV__PodSpeed();
			void vFCU_FCTL_MAINSM__EnterPreRun_Phase(Luint32 u32Key)

			if(u32PodSpeed < PODSPEED_STANDBY) && (sFCU.sStateMachine.eGSCommands == MAINSM_GS_ENTER_PRE_RUN_PHASE)
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

		//TODO: NOT MENTIONED ANYWHERE ELSE
		//process auto-sequence control
		vFCU_FCTL_MAINSM_AUTO__Process();

		//Check Operating States
		vFCU_FCTL_MAINSM__CheckIfUnlifted();

		vFCU_FCTL_MAINSM__CheckIfLifted();

		vFCU_FCTL_MAINSM__CheckIfHoveringStatically();

		vFCU_FCTL_MAINSM__CheckIfReadyForPush();

		vFCU_FCTL_MAINSM__CheckIfPushing();

		vFCU_FCTL_MAINSM__CheckIfCoasting();

		vFCU_FCTL_MAINSM__CheckIfBraking();

		vFCU_FCTL_MAINSM__CheckIfControlledBraking();
	}
	else
	{
		//do nothing.
	}


}

//allows us to enter pre-run phase from ethernet
void vFCU_FCTL_MAINSM__EnterPreRun_Phase(void)
{
	sFCU.sStateMachine.eGSCommand = MAINSM_GS_ENTER_PRE_RUN_PHASE;
}

void vFCU_FCTL_MAINSM__100MS_ISR(void)
{
	//Enable/Disable Counter
   if (sFCU.sStateMachine.EnableCounter == 1U)
   {
        sFCU.sStateMachine.Counter++;
   }
   else
   {
   		//do nothing
   }
}


/** OPERATING STATES */


void vFCU_FCTL_MAINSM__CheckIfUnlifted(void)
{

	//Determine if Lifted

	Luint32 u32PodZPos = s32FCU_FLIGHTCTL_LASERORIENT__Get_Z_Pos();

	if(u32PodZPos < C_FCU__LASERORIENT_MAX_UNLIFTED_HEIGHT)
	{
		sFCU.sStateMachine.sOpStates.u8Unlifted = 0U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8Unlifted = 1U;
	}
}

void vFCU_FCTL_MAINSM__CheckIfLifted(void)
{
	//Determine if Unlifted

	Luint32	u32PodZPos = s32FCU_FLIGHTCTL_LASERORIENT__Get_Z_Pos();

	if(u32PodZPos > C_FCU__LASERORIENT_MIN_LIFTED_HEIGHT)
	{
		sFCU.sStateMachine.sOpStates.u8Lifted = 1U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8Lifted = 0U;
	}
}


void vFCU_FCTL_MAINSM__CheckIfHoveringStatically(void)
{
	//Determine if Hovering Statically
	if(sFCU.sHoverengines.eState == HOVERENGINES_STATE__STATIC_HOVERING)
	{
		sFCU.sStateMachine.sOpStates.u8StaticHovering = 1U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8StaticHovering = 0U;
	}
}

void vFCU_FCTL_MAINSM__CheckIfReadyForPush(void)
{
	//Determine if Ready for Push

	//Get Pod Current Z Position
	Luint32 u32PodZPos = s16FCU_FLIGHTCTL_LASERORIENT__Get_Z_Pos();
	//Get Current Pod Speed
	Luint32 u32PodSpeed = u32FCU_FLIGHTCTL_NAV__PodSpeed();

	//Get values on checking the conditions for Pusher Interlock 
	//Get Pusher Interlock Switch 1 Status
	Luint8 u8PusherSwitch1 = u8FCU_PUSHER__Get_Switch(0);
	//Get Pusher Interlock Switch 2 Status
	Luint8 u8PusherSwitch2 = u8FCU_PUSHER__Get_Switch(1);

	//Check if we are connected to the pusher, speed is below standby, the height makes sense to be pushed + each of our landing gear units is retracted
	if((u32PodZPos > C_FCU_LASERORIENT_MIN_RUN_MODE_HEIGHT) &&  (u32PodSpeed < C_FCU__NAV_PODSPEED_STANDBY) && 	(u8PusherSwitch1 == 1U) && (u8PusherSwitch2 == 1U) && (vFCU_FLIGHTCTL_LIFTMECH__Get_State() == LIFT_MECH_STATE__RETRACTED))
	{
		sFCU.sStateMachine.sOpStates.u8ReadyForPush = 1U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8ReadyForPush = 0U;
	}
}

void vFCU_FCTL_MAINSM__CheckIfPushing(void)
{
	//Get Pusher Interlock Switch 1 Status
	Luint8 u8PusherSwitch1 = u8FCU_PUSHER__Get_Switch(0);
	//Get Pusher Interlock Switch 2 Status
	Luint8 u8PusherSwitch2 = u8FCU_PUSHER__Get_Switch(1);

	//Determine if in Pushing State
	if(u32PodSpeed > C_FCU__NAV_MIN_PUSHER_SPEED && (u8PusherSwitch1 == 1U) && (u8PusherSwitch2 == 1U))
	{
		sFCU.sStateMachine.sOpStates.u8Pushing = 1U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8Pushing = 0U;
	}
}

void vFCU_FCTL_MAINSM__CheckIfCoasting(void)
{
	//Get Pusher Interlock Switch 1 Status
	Luint8 u8PusherSwitch1 = u8FCU_PUSHER__Get_Switch(0);
	//Get Pusher Interlock Switch 2 Status
	Luint8 u8PusherSwitch2 = u8FCU_PUSHER__Get_Switch(1);

	//
	if(u32PodSpeed && (u8PusherSwitch1 == 0U) && (u8PusherSwitch2 == 0U) && (vFCU_FLIGHTCTL_BRAKES__Get_State() == BRAKES_STATE__RETRACTED))
	{
		sFCU.sStateMachine.sOpStates.u8Coast = 1U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8Coast = 0U;
	}
}

void vFCU_FCTL_MAINSM__CheckIfBraking(void)
{
	if(vFCU_FLIGHTCTL_BRAKES__Get_State() == BRAKES_STATE__BRAKING)
	{
		sFCU.sStateMachine.sOpStates.u8Braking = 1U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8Braking = 0U;
	}
}

void vFCU_FCTL_MAINSM__CheckIfControlledBraking(void)
{
	if(vFCU_FLIGHTCTL_BRAKES__Get_State() == BRAKES_STATE__BRAKING)
	{
		sFCU.sStateMachine.sOpStates.u8CtlEmergBraking = 1U;
	}
	else
	{
		sFCU.sStateMachine.sOpStates.u8CtlEmergBraking = 0U;
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

