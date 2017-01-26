/**
 * @file		FCU__MAIN_SM__AUTO_SEQ.C
 * @brief		Auto sequence functional tests.
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
 * @addtogroup FCU__MAIN_SM_AUTO
 * @ingroup FCU
 * @{ */

#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U

//the structure
extern struct _strFCU sFCU;

/***************************************************************************//**
 * @brief
 * Init the auto sequence system
 * 
 * @st_funcMD5		1551C289FFBD59697C35BA24059C46F1
 * @st_funcID		LCCM655R0.FILE.032.FUNC.001
 */
void vFCU_MAINSM_AUTO__Init(void)
{
	sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__RESET;

}


/***************************************************************************//**
 * @brief
 * Process the main state machine
 * 
 * @st_funcMD5		D7C64FED25A5476346946F581CF443BB
 * @st_funcID		LCCM655R0.FILE.032.FUNC.002
 */
void vFCU_MAINSM_AUTO__Process(void)
{
	Luint8 u8Counter;
	Luint8 u8IsCalibrationDone;
	Lfloat32 f32IBeam_Left_mm;
	Lfloat32 f32IBeam_Right_mm;
	Luint8 u8IsActionSuccessful;			// This will change in every action state
	Luint8 u8IsSensorReadSuccessful;		// This will change in every sensor read state

	// this is STATIC, BE CAREFUL. MAKE SURE YOU CLEAR IT ONCE THE USAGE IS DONE
	// 0 -> not requested, 1 -> requested
	static Luint8 u8IsActionRequested = 0;


#if C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
	E_FCU__SWITCH_STATE_T eSwitchState_Left_Extend;
	E_FCU__SWITCH_STATE_T eSwitchState_Left_Retract;
	E_FCU__SWITCH_STATE_T eSwitchState_Right_Extend;
	E_FCU__SWITCH_STATE_T eSwitchState_Right_Retract;
#endif // C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1

	//handle the state machine.
	switch(sFCU.sAutoSequence.eAutoSeqState)
	{

		case AUTOSEQ_STATE__RESET:
			//we have just come out of reset here.

			//change state.
			sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__IDLE;
			break;

		case AUTOSEQ_STATE__IDLE:

			// You either START or SKIP test phase here.
			// The packet that controls eAutoSeqControl comes from GS.
			switch(sFCU.sAutoSequence.eAutoSeqControl)
			{
				case AUTOSEQ_TEST_NOT_START:
					// Don't do any thing. Stay in AUTOSEQ_STATE__IDLE
					break;

				case AUTOSEQ_TEST_START:
					// ground station has asked to start.
					// check if all calibration is already done and only then
					// start testing

					// First routine is checking brakes.
					u8IsCalibrationDone = u8FCU_BRAKES_CAL__Is_Busy();

					if(u8IsCalibrationDone)
					{
						// todo For now, just move to done for testing
						sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_DONE;
					}
					else
					{
						// Nothing to do, stay here, calibration needs more time
					}
					break;

				case AUTOSEQ_TEST_SKIP:

					// Auto sequence routine completely skipped, can be done only
					// through a ground station command.
					sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_DONE;
					break;

				case AUTOSEQ_TEST_KILL:

					// Auto sequence routine completely killed, can be done only
					// through a ground station command.
					sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_DONE;
					break;

				default:
					// Nothing to do
					break;
			}

			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKES_INIT_ACTION:

			// If we are in this state for the first time
			if(u8IsActionRequested == 0)
			{
				// THEN,
				// Move the brakes to fully retract state
				u8IsActionRequested = u8FCU_MAINSM_AUTO_Brakes_Init_Action();
			}
			else
			{
				// we have already made the request. Let's wait for the actuation to finish
				u8IsActionSuccessful = u8FCU_BRAKES__Is_Brake_Movement_Done();
				if (u8IsActionSuccessful == 1)
				{
					sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKES_INIT_EXPECTED_RESULT;
				}
			}

			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKES_INIT_EXPECTED_RESULT:

			// In AUTOSEQ_STATE__TEST_FUNCTION_BRAKES_INIT_ACTUATION we moved the brakes
			// to fully retracted if they were not there already.

			// Check the retract limit switches for both left and right eddy brakes.
			// If the upper state machine is working properly, then by the time we
			// come here global switch states would have been updated
#if C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
			eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
			eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

			// Don't go to the next state until the brakes are retracted and limit
			// switches are closed
			if (eSwitchState_Left_Retract == SW_STATE__CLOSED && eSwitchState_Right_Retract == SW_STATE__CLOSED)
			{
#endif // C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u

				// Stepper motor, Limit switch and MLP show we are retracted.
				// Let's go to next test case
				sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_HALF_WAY_ACTION;
#if C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
			}
			else
			{
				// Stay here till time out happens
				// todo Add a timeout here
			}
#endif // C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_HALF_WAY_ACTION:

			// Make sure again, brakes are retracted and the retract limit
			// switches reflect the same state
			// Get the position of the left and right brake
			f32IBeam_Left_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__LEFT);
			f32IBeam_Right_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__RIGHT);

			// Add the tolerance to compensate for any possible error from sensors
			f32IBeam_Left_mm += C_FCU__BRAKES__IBEAM_DIST_TOLERANCE_MM;
			f32IBeam_Right_mm += C_FCU__BRAKES__IBEAM_DIST_TOLERANCE_MM;

			// IF, Both the brakes distance from the Ibeam + tolerance because of
			// the sensor/ADC error should be max i.e. fully retracted
			if((f32IBeam_Left_mm >= C_FCU__BRAKES__MAX_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Left_mm <= C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM))
			{
				// Yes, THEN, check if right brake is fully retracted, account for the errors from the
				// sensor read
				if((f32IBeam_Right_mm >= C_FCU__BRAKES__MAX_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Right_mm <= C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM))
				{
					eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
					eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

					// AND, retract limit switches for both left and right brakes
					// should be closed
					if (eSwitchState_Left_Retract == SW_STATE__CLOSED
							&& eSwitchState_Right_Retract == SW_STATE__CLOSED)
					{
						// THEN, Request the brakes to move halfway
						vFCU_BRAKES__Move_IBeam_Distance_mm(C_FCU__BRAKES__HALF_WAY_IBEAM_DIST_MM);

						// Brakes asked to move half way. This won't happen immediately.
						// Let's go to the next state to read the sensors for results
						sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_HALF_WAY_EXPECTED_RESULT;
					}
				}
			}
			else
			{
				// Stay in this until a time out
				// todo design a time out.
			}

			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_HALF_WAY_EXPECTED_RESULT:
			// In AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_HALF_WAY_ACTION we moved the brakes
			// half way between the fully retracted and extended position

			// Get the position of the left and right brake
			f32IBeam_Left_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__LEFT);
			f32IBeam_Right_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__RIGHT);

			// If the upper level state machine is working properly, then by the time we
			// come here global switch states would have been updated
			eSwitchState_Left_Extend = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
			eSwitchState_Right_Extend = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

			// None of the limit switches for left or right brake should be open,
			// since the brakes have started moving inwards away from all the limit
			// switches
			if (eSwitchState_Right_Extend == SW_STATE__OPEN
					&& eSwitchState_Right_Retract == SW_STATE__OPEN
					&& eSwitchState_Left_Extend == SW_STATE__OPEN
					&& eSwitchState_Left_Retract == SW_STATE__OPEN)
			{
				// All the limit switches open --> Good
				// Now, check the distance of the left brake from the IBeam
				// It should be somewhere halfway between fully retract and extend
				if ((f32IBeam_Left_mm >= C_FCU__BRAKES__HALF_WAY_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Left_mm <= C_FCU__BRAKES__HALF_WAY_IBEAM_DIST_UP_BOUND_MM))
				{
					// Yes, THEN, check the distance of right brake from the IBeam
					// It should be somewhere halfway between fully retract and extend
					if ((f32IBeam_Right_mm >= C_FCU__BRAKES__HALF_WAY_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Right_mm <= C_FCU__BRAKES__HALF_WAY_IBEAM_DIST_UP_BOUND_MM))
					{
						// THEN, everything looks good. Let's move to the next test phase
						sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_EXTEND_ACTION;
					}
				}
			}
			else
			{
				// Stay here till time out happens
				// todo Add a timeout here
			}

			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_EXTEND_ACTION:

			// In the last "action", we moved the brakes half way
			// Before we start this action, check if all the limit switches are open again
			eSwitchState_Left_Extend = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
			eSwitchState_Right_Extend = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

			// None of the limit switches for left or right brake should be open,
			// since the brakes have started moving inwards away from all the limit
			// switches
			if (eSwitchState_Right_Extend == SW_STATE__OPEN
					&& eSwitchState_Right_Retract == SW_STATE__OPEN
					&& eSwitchState_Left_Extend == SW_STATE__OPEN
					&& eSwitchState_Left_Retract == SW_STATE__OPEN)
			{
				// THEN, Request the brakes to extend completely
				vFCU_BRAKES__Move_IBeam_Distance_mm(C_FCU__BRAKES__MIN_IBEAM_DIST_MM);

				// The deed is done, let's go to the next state to check if we
				// are fully extended
				sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_EXTEND_EXPECTED_RESULT;
			}
			else
			{
				// Stay here till time out happens
				// todo Add a timeout here
			}

			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_EXTEND_EXPECTED_RESULT:

			// In the previous state we tried to move the brakes to fully
			// extend position
			eSwitchState_Left_Extend = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
			eSwitchState_Right_Extend = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

			// By the time the previous action completes, the left and right
			// brake extend switches should be close and the retract limit
			// switches for both the brakes should be open
			if (eSwitchState_Right_Extend == SW_STATE__CLOSED
					&& eSwitchState_Left_Extend == SW_STATE__CLOSED
					&& eSwitchState_Right_Retract == SW_STATE__OPEN
					&& eSwitchState_Left_Retract == SW_STATE__OPEN)
			{
				// Yes, THEN check the MLP reading to find the distance of the
				// brakes from IBeam. It should be the minimum value allowed i.e.
				// as close to the IBeam as possible
				// Get the position of the left and right brake
				f32IBeam_Left_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__LEFT);
				f32IBeam_Right_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__RIGHT);

				// Check if left brake is fully extended, account for the errors from the
				// sensor read
				if((f32IBeam_Left_mm >= C_FCU__BRAKES__MIN_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Left_mm <= C_FCU__BRAKES__MIN_IBEAM_DIST_UP_BOUND_MM))
				{
					// Yes, THEN, check if right brake is fully extended, account for the errors from the
					// sensor read
					if((f32IBeam_Right_mm >= C_FCU__BRAKES__MIN_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Right_mm <= C_FCU__BRAKES__MIN_IBEAM_DIST_UP_BOUND_MM))
					{
						// Yes, THEN, we are good to go for the next state.
						sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_RETRACT_ACTION;
					}
				}
			}
			else
			{
				// Stay here till time out happens
				// todo Add a timeout here
			}

			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_RETRACT_ACTION:

			// In the previous state we tried to move the brakes to fully
			// extended position
			eSwitchState_Left_Extend = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
			eSwitchState_Right_Extend = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

			// By the time the previous action completes, the left and right
			// brake extend switches should be CLOSED and the retract limit
			// switches for both the brakes should be OPEN
			if (eSwitchState_Right_Extend == SW_STATE__CLOSED
					&& eSwitchState_Left_Extend == SW_STATE__CLOSED
					&& eSwitchState_Right_Retract == SW_STATE__OPEN
					&& eSwitchState_Left_Retract == SW_STATE__OPEN)
			{
				// THEN, Request the brakes to retract completely
				vFCU_BRAKES__Move_IBeam_Distance_mm(C_FCU__BRAKES__MIN_IBEAM_DIST_MM);

				// The deed is done, let's go to the next state to check if we
				// are fully extended
				sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_RETRACT_EXPECTED_RESULT;
			}
			else
			{
				// Stay here till time out happens
				// todo Add a timeout here
			}

			break;

		case AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_RETRACT_EXPECTED_RESULT:

			// In the previous state we tried to move the brakes to fully
			// retract position
			eSwitchState_Left_Extend = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
			eSwitchState_Right_Extend = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__EXTEND].eSwitchState;
			eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

			// By the time the previous action completes, the left and right
			// brake extend switches should be OPEN and the retract limit
			// switches for both the brakes should be CLOSED
			if (eSwitchState_Right_Extend == SW_STATE__OPEN
					&& eSwitchState_Left_Extend == SW_STATE__OPEN
					&& eSwitchState_Right_Retract == SW_STATE__CLOSED
					&& eSwitchState_Left_Retract == SW_STATE__CLOSED)
			{
				// Yes, THEN check the MLP reading to find the distance of the
				// brakes from IBeam. It should be the max value allowed i.e.
				// as far from the IBeam as possible
				// Get the position of the left and right brake
				f32IBeam_Left_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__LEFT);
				f32IBeam_Right_mm = f32FCU_BRAKES__Get_IBeam_mm(FCU_BRAKE__RIGHT);

				// Check if left brake is fully retracted, account for the errors from the
				// sensor read
				if((f32IBeam_Left_mm >= C_FCU__BRAKES__MAX_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Left_mm <= C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM))
				{
					// Yes, THEN, check if right brake is fully retracted, account for the errors from the
					// sensor read
					if((f32IBeam_Right_mm >= C_FCU__BRAKES__MAX_IBEAM_DIST_LOW_BOUND_MM) && (f32IBeam_Right_mm <= C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM))
					{
						// Yes, THEN, we are good to go for the next state.
						// FOR NOW, this finishes our auto sequence testing
						sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__IDLE;
					}
				}
			}
			else
			{
				// Stay here till time out happens
				// todo Add a timeout here
			}


		case AUTOSEQ_STATE__TEST_DONE:

			// you can be here if the test phase was
			// 1) Successful
			// 2) Failed
			// 3) Skipped entirely
			// 4) Killed from ground station



			break;


		default:
			//should not get here
			break;

	}//switch(sFCU.eRunState)
}


/***************************************************************************//**
 * @brief
 * Are we busy in the auto sequence
 * 
 * @return			0 = not busy, 1 = busy
 * @st_funcMD5		AC6581920B5F232BBDE88D0B8D60E7AB
 * @st_funcID		LCCM655R0.FILE.032.FUNC.003
 */
Luint8 u8FCU_MAINSM_AUTO__Is_Busy(void)
{
	Luint8 u8Return;

	if(sFCU.sAutoSequence.eAutoSeqState == AUTOSEQ_STATE__TEST_DONE)
	{
		u8Return = 0U;
	}
	else
	{
		u8Return = 1U;
	}

	return u8Return;
}


/***************************************************************************//**
 * @brief
 * Do we need to do an immediate abort?
 * 
 * @return			1 = Immediate abort, 0 = no issue
 * @st_funcMD5		A4431A735C6F0C1AA502E235E9BEC53B
 * @st_funcID		LCCM655R0.FILE.032.FUNC.004
 */
Luint8 u8FCU_MAINSM_AUTO__Is_Abort(void)
{
	return sFCU.sAutoSequence.u8IsAutoSequenceAbort;
}

/***************************************************************************//**
 * @brief
 * In the init action for brakes we move the brakes to fully retract position
 *
 * @return			1 = Action Successful, 0 = Action failed
 * @st_funcMD5
 * @st_funcID
 */
Luint8 u8FCU_MAINSM_AUTO_Brakes_Init_Action(void)
{
	Luint8 u8Return = 0;
	Lfloat32 f32MLP_LeftBrakePosition_mm;
	Lfloat32 f32MLP_RightBrakePosition_mm;
	Luint8 u8Test = 0; // To check if a request was successful; 1-> success, 0 -> fail

	// Initialize the variables to 0 before test
	f32MLP_LeftBrakePosition_mm = 0.0;
	f32MLP_RightBrakePosition_mm = 0.0;

	// Get the position of the left and right brake
	f32MLP_LeftBrakePosition_mm = f32FCU_BRAKES_MLP__Get_Brake_Position(FCU_BRAKE__LEFT);
	f32MLP_RightBrakePosition_mm = f32FCU_BRAKES_MLP__Get_Brake_Position(FCU_BRAKE__RIGHT);

	// If either brakes are not completely retracted
	if((f32MLP_LeftBrakePosition_mm > C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM)
			|| (f32MLP_RightBrakePosition_mm > C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM))
	{

		u8Test = u8FCU_BRAKES__Is_Brake_Movement_Possible();

		// If brakes state machine can take new brake move requests
		if(u8Test == 1)
		{
			// Then Retract both the brakes to fully retracted position
			vFCU_BRAKES__Move_IBeam_Distance_mm(C_FCU__BRAKES__MAX_IBEAM_DIST_MM);

			// Request made successfully
			u8Return = 1;
		}
		else
		{
			// Can't make this request, brakes are busy,
			// TRY AGAIN, NOT IMMEDIATELY, we will come here again from higher state machines
			u8Return = 0;
		}
	}

	return u8Return;
}


Luint8 u8FCU_MAINSM_AUTO_Brakes_Init_Is_Expected_Result(void)
{
	Luint8 u8Return = 0;
#if C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
	E_FCU__SWITCH_STATE_T eSwitchState_Left_Extend;
	E_FCU__SWITCH_STATE_T eSwitchState_Left_Retract;
	E_FCU__SWITCH_STATE_T eSwitchState_Right_Extend;
	E_FCU__SWITCH_STATE_T eSwitchState_Right_Retract;
#endif // C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1
	Lfloat32 f32MLP_LeftBrakePosition_mm;
	Lfloat32 f32MLP_RightBrakePosition_mm;

	// Initialize the variables to 0 before test
	f32MLP_LeftBrakePosition_mm = 0.0;
	f32MLP_RightBrakePosition_mm = 0.0;

	// In AUTOSEQ_STATE__TEST_FUNCTION_BRAKES_INIT_ACTUATION we moved the brakes
	// to fully retracted if they were not there already.

	// Get the position of the left and right brake
	f32MLP_LeftBrakePosition_mm = f32FCU_BRAKES_MLP__Get_Brake_Position(FCU_BRAKE__LEFT);
	f32MLP_RightBrakePosition_mm = f32FCU_BRAKES_MLP__Get_Brake_Position(FCU_BRAKE__RIGHT);

	// Check the retract limit switches for both left and right eddy brakes.
	// If the upper state machine is working properly, then by the time we
	// come here global switch states would have been updated
#if C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
	eSwitchState_Left_Retract = sFCU.sBrakes[FCU_BRAKE__LEFT].sLimits[BRAKE_SW__RETRACT].eSwitchState;
	eSwitchState_Right_Retract = sFCU.sBrakes[FCU_BRAKE__RIGHT].sLimits[BRAKE_SW__RETRACT].eSwitchState;

	// Don't go to the next state until the brakes are retracted and limit
	// switches are closed
	if (eSwitchState_Left_Retract == SW_STATE__CLOSED && eSwitchState_Right_Retract == SW_STATE__CLOSED)
	{
#endif // C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u

		// If either brakes are not completely retracted
		if((f32MLP_LeftBrakePosition_mm > C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM)
				&& (f32MLP_RightBrakePosition_mm > C_FCU__BRAKES__MAX_IBEAM_DIST_UP_BOUND_MM))
		{
			// Stepper motor, Limit switch and MLP show we are retracted.
			// Let's go to next test case

#if C_LOCALDEF__LCCM655__ENABLE_FCTL_BRAKES_HALFWAY_ACTUATION == 1
			sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_HALF_WAY_ACTION;
#else
			sFCU.sAutoSequence.eAutoSeqState = AUTOSEQ_STATE__TEST_FUNCTION_BRAKE_FULL_RETRACT_ACTION;
#endif // C_LOCALDEF__LCCM655__ENABLE_FCTL_BRAKES_HALFWAY_ACTUATION == 1
		}

#if C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
	}
#endif // C_LOCALDEF__LCCM655__ENABLE_FCTL_LIMIT_SWITCH_ACCESS == 1u
	else
	{
		//
	}

	return u8Return;
}


#endif //#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
//safetys
#ifndef C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE
	#error
#endif
/** @} */
/** @} */
/** @} */

