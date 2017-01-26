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

#define ACCELERATION_ARRAY_SIZE = 10
#define MAX_LONGETIUDENAL_POSITION_USE_ACCELEROMETER = 

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

	sFCU.sFlightControl.sNavigation.s16Roll = 0;
	sFCU.sFlightControl.sNavigation.s16Pitch = 0;
	sFCU.sFlightControl.sNavigation.s16Yaw = 0;

	// TODO check if accelerometers are working?
	
	// initialise arrays for filtering purposes
	sFCU.sFlightControl.sNavigation.f32AccelerationArray[ACCELERATION_ARRAY_SIZE] = {0};

}


/***************************************************************************//**
 * @brief
 * Process the calculation of pod NAVIGATION
 * 
 */
void vFCU_FLIGHTCTL_NAVIGATION__Process(void)
{
	sFCU.sFlightControl.sNavigation.s16Roll = s16FCU_FLIGHTCTL_ORIENTATION__Get_Roll();
	sFCU.sFlightControl.sNavigation.s16Pitch = s16FCU_FLIGHTCTL_ORIENTATION__Get_Pitch();
	sFCU.sFlightControl.sNavigation.s16Yaw = s16FCU_FLIGHTCTL_ORIENTATION__Get_Yaw();

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
	// use pitch, yaw and roll along with accelerometer data to get corrected acceleration
	Lfloat32 f32LongitudenalAcceleration;
	if (sFCU.sFlightControl.sNavigation.f32LongitudenalPosition < MAX_LONGETIUDENAL_POSITION_USE_ACCELEROMETER)
	{	
		// get filtered accelerometer value. need to confirm how we get the data
		Lfloat32 f32Accel1 = sFCU.sFlightControl.sAccelerometer[0].getAccelerationFiltered();
		Lfloat32 f32Accel2 = sFCU.sFlightControl.sAccelerometer[1].getAccelerationFiltered();

		// compute final longitudenal acceleration as the mean of the acceleration from valid accelerometers
		f32LongitudenalAcceleration = (f32Accel1[0] + f32Accel2[0] + f32Accel1[1] + f32Accel2[1] + f32Accel1[2] + f32Accel2[2]) / 6;
	}
	else
	{
		// depending on position down the tube, use laser range finder to compute acceleration
		f32LongitudenalAcceleration = getAccelerationFromLaserRangeFinder();
	}

	// TODO compensate for thermal drift?
}



/****************************************************************************/
/** Functions to retrieve NAVIGATION parameters, to be called from other files */



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
