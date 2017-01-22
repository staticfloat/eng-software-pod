 /**
 * @file		FCU__NAVIGATION.c
 * @brief		Determine Pod Front and Rear x Position, Speed and Acceleration in tube
 * @author		Paul Le Henaff
 * @copyright	rLoop Inc.
 */
/**
 * @addtogroup RLOOP
 * @{ */
/**
 * @addtogroup FCU
 * @ingroup RLOOP
 * @{ */


// All units in mm, but the math doesn't care as long as you're consistent


#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FCTL_NAVIGATION == 1U

//the structure
extern struct _strFCU sFCU;

//locals
static void vFCU_FLIGHTCTL_NAVIGATION__CalcLongitudenalPosition(void);
static void vFCU_FLIGHTCTL_NAVIGATION__CalcLongitudenalSpeed(void);
static void vFCU_FLIGHTCTL_NAVIGATION__CalcLongitudenalAcceleration(void);


/***************************************************************************//**
 * @brief
 * Init any navigation variables, etc.
 * 
 *
 */
void vFCU_FLIGHTCTL_NAVIGATION__Init(void)
{
	// TODO add this structure to sFCU
	sFCU.sFlightControl.sNavigation.f32LongitudenalPosition = 0; // distance along tube
	sFCU.sFlightControl.sNavigation.f32LongitudenalAcceleration = 0; // acceleration
	sFCU.sFlightControl.sNavigation.f32LongitudenalVelocity = 0; // velocity
}


/***************************************************************************//**
 * @brief
 * Process the calculation of pod NAVIGATION
 * 
 */
void vFCU_FLIGHTCTL_NAVIGATION__Process(void)
{
	Luint8 u8OperationalLasers[C_FCU__NUM_LASERS_GROUND];
	Luint8 u8OperationalCount = 0U;
	Luint8 u8Counter;


	//handle the state machine
	switch(sFCU.sFlightControl.sOrient.eState)
	{
		case LASER_NAVIGATION_STATE__IDLE:
			//do nothing
			break;

		case LASER_NAVIGATION_STATE__INIT:
			//do nothing?
			sFCU.sFlightControl.sOrient.eState = LASER_NAVIGATION_STATE__GET_LASER_DATA;
			break;

		case LASER_NAVIGATION_STATE__GET_LASER_DATA:
			/** store each laser's most recent distance measurement and error state to sOrient struct */

			for(u8Counter = 0U; u8Counter < C_FCU__NUM_LASERS_GROUND; u8Counter++)
			{
				sFCU.sFlightControl.sOrient.sGroundLasers[u8Counter].f32Measurement = f32FCU_LASEROPTO__Get_Distance(u8Counter);
				sFCU.sFlightControl.sOrient.sGroundLasers[u8Counter].u8Error = u8FCU_LASEROPTO__Get_Error(u8Counter);
			} 

			for(u8Counter = 0U; u8Counter < C_FCU__NUM_LASERS_IBEAM; u8Counter++)
			{
				sFCU.sFlightControl.sOrient.sBeamLasers[u8Counter].f32Measurement = f32FCU_LASEROPTO__Get_Distance(u8Counter + C_FCU__NUM_LASERS_GROUND); // kinda hacky; consistency in laser/HE numbering should help ground this
				sFCU.sFlightControl.sOrient.sBeamLasers[u8Counter].u8Error = u8FCU_LASEROPTO__Get_Error(u8Counter + C_FCU__NUM_LASERS_GROUND);
			} 

			sFCU.sFlightControl.sOrient.eState = LASER_NAVIGATION_STATE__RECALCULATE_PITCH_ROLL_TWIST;
			break;

		case LASER_NAVIGATION_STATE__RECALCULATE_PITCH_ROLL_TWIST:
			/** count which lasers are not in the error state and append them to array */

			for(u8Counter = 0U; u8Counter < C_FCU__NUM_LASERS_GROUND; u8Counter++)
			{
				if(sFCU.sFlightControl.sOrient.sGroundLasers[u8Counter].u8Error != 1U)
				{
					// Laser works, store its index in the array
					u8OperationalLasers[u8OperationalCount] = u8Counter; 
					// increment count of operational lasers
					u8OperationalCount += 1U; 
				}
				else
				{
					// bad laser; dont append its index to array
				}
			}

			/** Calculate as many of the pods NAVIGATION parameters as possible based on the number of operational lasers */
			if(u8OperationalCount == 4U)
			{
				// calculate pitch, roll, and twist of the pod
				// 1st triplet of ground lasers
			    vFCU_FLIGHTCTL_NAVIGATION__CalculateGroundPlane(0U, 1U, 2U, &sFCU.sFlightControl.sOrient.f32PlaneCoeffs[0]);

			    // 2nd triplet of ground lasers
			    vFCU_FLIGHTCTL_NAVIGATION__CalculateGroundPlane(1U, 2U, 3U, &sFCU.sFlightControl.sOrient.f32TwistPlaneCoeffs[0]);

			    for(u8Counter = 0U; u8Counter < C_FCU__NUM_HOVER_ENGINES; u8Counter++)
			    {
			    	// Calc the position of each hover engine
					sFCU.sFlightControl.sOrient.sHoverEngines[u8Counter].f32Measurement = f32FCU_FLIGHTCTL_NAVIGATION__PointToPlaneDistance(&sFCU.sFlightControl.sOrient.sHoverEngines[u8Counter].f32Position[0]);
				}

				vFCU_FLIGHTCTL_NAVIGATION__CalcRoll();
				vFCU_FLIGHTCTL_NAVIGATION__CalcPitch();

				vFCU_FLIGHTCTL_NAVIGATION__CalcTwistRoll();
				vFCU_FLIGHTCTL_NAVIGATION__CalcTwistPitch();
			}
			else if(u8OperationalCount == 3U)
			{ 
				// calculate pitch and roll. cannot calculate twist.
			    vFCU_FLIGHTCTL_NAVIGATION__CalculateGroundPlane(u8OperationalLasers[0], u8OperationalLasers[1], u8OperationalLasers[2], &sFCU.sFlightControl.sOrient.f32PlaneCoeffs[0]);

			    for(u8Counter = 0U; u8Counter < C_FCU__NUM_HOVER_ENGINES; u8Counter++)
			    {
			    	// Calc the position of each hover engine
					sFCU.sFlightControl.sOrient.sHoverEngines[u8Counter].f32Measurement = f32FCU_FLIGHTCTL_NAVIGATION__PointToPlaneDistance(&sFCU.sFlightControl.sOrient.sHoverEngines[u8Counter].f32Position[0]);
				}

				vFCU_FLIGHTCTL_NAVIGATION__CalcPitch();
				vFCU_FLIGHTCTL_NAVIGATION__CalcRoll();
			}
			else if(u8OperationalCount == 2U)
			{
				// there are 2 operable lasers; can't compute any of the NAVIGATION parameters directly, but can infer information from what we have.
				// TODO write code to infer NAVIGATION parameters
			}
			else
			{
				// there is 1 or fewer operable lasers; can't compute twist/pitch/roll/HE heights.
			}

			sFCU.sFlightControl.sOrient.eState = LASER_NAVIGATION_STATE__RECALCULATE_YAW_AND_LATERAL;
			break;

		case LASER_NAVIGATION_STATE__RECALCULATE_YAW_AND_LATERAL:

			/** count which lasers are not in the error state and append them to array. */
			Luint8 u8OperationalCount = 0U;
			Luint8 u8OperationalLasers[C_FCU__NUM_LASERS_IBEAM];

			for(Luint8 u8Counter = 0U; u8Counter < C_FCU__NUM_LASERS_IBEAM; u8Counter++)
			{
				if(sFCU.sFlightControl.sOrient.sBeamLasers[u8Counter].u8Error != 1U)
				{
					// Laser works, store its index in the array
					u8OperationalLasers[u8OperationalCount] = u8Counter; 
					// increment count of operational lasers
					u8OperationalCount += 1U; 
				}
				else
				{
					// bad laser; dont append its index to array
				}
			}

			/** Calculate as many of the pods NAVIGATION parameters as possible based on the number of operational lasers */
			if(u8OperationalCount == 2U)
			{
				vFCU_FLIGHTCTL_NAVIGATION__CalcYaw_and_Lateral();

			}
			else if(u8OperationalCount == 1U)
			{
				// At least one i-beam laser is down; can't explicitly compute yaw/translation.
				//TODO: assume yaw = 0 compute lateral translation?  or the opposite, depending on which parameter is more likely to deviate from its ideal value.
			}
			else 
			{
				// no i-beam lasers are working, no measurement can be made.
			}

			sFCU.sFlightControl.sOrient.eState = LASER_NAVIGATION_STATE__INIT;
			break;

		case LASER_NAVIGATION_STATE__WAIT_LOOPS:
			// do nothing
			break;

		case LASER_NAVIGATION_STATE__ERROR:
			//some error has happened 
			break;
	}
}

/** The longitudenal position down the tube */
/*
input
=====							
contrast sensor 1 value
contrast sensor 2 value
contrast sensor 3 value
laser range finder value
speed
speed uncertainty
speed validity
*/

/*
output
======
front x position (x position of pod nose in the tube)
rear x position (x position of pod pusher plate in the tube)
x position validity
*/
void vFCU_FLIGHTCTL_NAVIGATION__CalcLongitudenalPosition(void)
{
	
}

/** The longitudenal position down the tube */
/*
input
=====
contrast sensor 1 value
contrast sensor 2 value
contrast sensor 3 value
laser range finder value
acceleration
acceleration uncertainty
acceleration validity
*/

/*
output
======
speed
speed uncertainty
speed validity
*/
void vFCU_FLIGHTCTL_NAVIGATION__CalcSpeed(void)
{
	
}

/** The longitudenal position down the tube */
/*
input
=====
accelerometer 1 value
accelerometer 2 value
pitch
roll
yaw
*/

/*
output
======
acceleration
acceleration uncertainty
acceleration validity
*/
void vFCU_FLIGHTCTL_NAVIGATION__CalcAcceleration(void)
{
	
}



/****************************************************************************/
/** Functions to retrieve NAVIGATION parameters, to be called from other files */

/** Get pod's current Roll */
Lint16 s16FCU_FLIGHTCTL_NAVIGATION__Get_Roll()
{
	return sFCU.sFlightControl.sOrient.s16Roll
}


#endif //C_LOCALDEF__LCCM655__ENABLE_FCTL_NAVIGATION
#ifndef C_LOCALDEF__LCCM655__ENABLE_FCTL_NAVIGATION
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
