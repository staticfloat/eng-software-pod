/**
* @file       FCU__FCTL__BRAKE_PID.C
* @brief      PID to control brake profile
* @author	  Nazneen Khan, Sean, Lachlan Grogan
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
 * @addtogroup FCU__FLIGHT_CTL__BRAKEPID
 * @ingroup FCU
 * @{ */


#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U
#if C_LOCALDEF__LCCM655__ENABLE_BRAKEPID == 1U

extern struct _strFCU sFCU;


// TODO: add struct to fcu_core.h inside of _strFCU:
#if C_LOCALDEF__LCCM655__ENABLE_BRAKEPID == 1U
struct
{
	Lfloat32 s32LastError;

}sBrakesPid;
#endif


void vFCU_FLIGHTCTL_BRAKEPID__Init(void)
{
	// set initial conditions for pid
	sBrakesPid.s32LastError=0;

}


void vFCU_FLIGHTCTL_BRAKEPID__Process(void)
{
	Lint32 kp, ki, kd;
	Lfloat32 s32MeasDistance, s32MeasVelocity, s32SetpointVelocity;
	Lfloat32 s32Error, s32LastError;
	Lfloat32 s32Integral, s32Derivative;
	Lfloat32 s32BrakekGap, s32BrakePos;


	// measured distance and scale it appropriately so can use as index
	s32MeasDistance = (Lfloat32)u32FCU_FLIGHTCTL_XXXXXX__GetFrontPos();
	// scale it so can use as index

	// look up setpoint velocity
	s32SetpointVelocity = f32A34_SetpointVelocityTable[s32MeasDistance];

	// measure velocity
	s32MeasVelocity = (Lfloat32)u32FCU_FLIGHTCTL_XXXXX__PodSpeed();
	// scale it so can use as index

	// compute error
	// TODO: check scaling
	s32Error = s32SetpointVelocity - s32MeasVelocity;

	// compute integral
	s32Integral = s32Integral + s32Error

	// compute derivative
	s32Derivative = s32Error - s32LastError;

	// look up pid gains
	kp = f32A34_GainsTable[s32MeasVelocity];
	ki = f32A34_GainsTable[s32MeasVelocity + 1U];
	kd = f32A34_GainsTable[s32MeasVelocity + 2U];

	// calculate brake position increment (brake gap) from pid equations
	// TODO: verify this represents the equation
	// u = [ kp + ki * 1/s + kd * (s/(Tf*s + 1) ] * e
	s32BreakGap = (kp * s32Error) + (ki * s32Integral) + (kd * s32Derivative);

	// get current brake position
	s32BrakePos = (s32FCU_BRAKES__Get_CurrentPos(FCU_BRAKE__LEFT ) + s32FCU_BRAKES__Get_CurrentPos(FCU_BRAKE__LEFT )) >> 1U;

	// add brake gap to current brake position
	s32BrakePos += s32BreakGap;

	// set new brake position
	// TODO: check units, need in micros for this function
	vFCU_BRAKES_STEP__Move(s32BrakePos, s32BrakePos);

	// save current error
	s32LastError = s32Error;



}





#endif //C_LOCALDEF__LCCM655__ENABLE_BRAKEPID
#ifndef C_LOCALDEF__LCCM655__ENABLE_BRAKEPID
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

