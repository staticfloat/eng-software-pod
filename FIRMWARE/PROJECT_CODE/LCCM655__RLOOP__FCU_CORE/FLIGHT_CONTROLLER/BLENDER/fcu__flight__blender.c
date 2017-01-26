//Blends the position, accel and veloc from lower layers and gives a
//single point of interface to the flight layer with our primary measurements
//also allows the track profile to dictate what sensors are used for what measurements.


//Laser distance
//void vFCU_LASERDIST__Init(void);
//void vFCU_LASERDIST__Process(void);
//Lint32 s32FCU_LASERDIST__Get_Distance_mm(void);
//Lint32 s32FCU_LASERDIST__Get_Velocity_mms(void);
//void vFCU_LASERDIST__100MS_ISR(void);



#include "../../fcu_core.h"

#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
#if C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL == 1U

//the structure
extern struct _strFCU sFCU;

//return the current values
Lint32 s32FCU_FCTL_BLENDER__Get_Accel_mmss(void)
{
	return sFCU.sFlightControl.sBlender.s32Accel_mmss;
}

Lint32 s32FCU_FCTL_BLENDER__Get_Veloc_mms(void)
{
	return sFCU.sFlightControl.sBlender.s32Veloc_mms;
}

Lint32 s32FCU_FCTL_BLENDER__Get_Displacement_mm(void)
{
	return sFCU.sFlightControl.sBlender.s32Disp_mm;
}



void vFCU_FCTL_BLENDER__Init(void)
{
	//todo: init flight conditions
	sFCU.sFlightControl.sBlender.sWorking.s32Accel_mmss = 0;
	sFCU.sFlightControl.sBlender.sWorking.s32Veloc_mms = 0;
	sFCU.sFlightControl.sBlender.sWorking.s32Disp_mm = 0;
	sFCU.sFlightControl.sBlender.sWorking.u8LaserRangeFinder_Valid = 0U;
	sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter = 0;
	sFCU.sFlightControl.sBlender.sWorking.u8TimeCounterEnable = 0U;
}


void vFCU_FCTL_BLENDER__Process(void)
{
	//todo
	//determine from the track database what sensors we need
	//add in any geometry offsets
	//check things like laser range as its not a displacement from the start of the track
	//determine 3 current values, A, V, D

	if(sFCU.sFlightControl.sBlender.sWorking.s32Disp_mm < MIN_DISPLACEMENT_TO_USE_LASER_RANGE_FINDER)
	//  front x position is lower than min position to use laser range finder,
	{
		if(sFCU.sFlightControl.sBlender.sWorking.u8LaserRangeFinder_Valid > 0U)
		{
			sFCU.sFlightControl.sBlender.sWorking.u8LaserRangeFinder_Valid = 0U;
		}
	}
	else
	//  front x position is higher than min position to use laser range finder,
	{
		if((sFCU.sFlightControl.sBlender.sWorking.u8LaserRangeFinder_Valid == 0U)
		&& (sFCU.sFlightControl.sBlender.sDisplacement_m.sfromLaser.u32Value > LASER_RANGE_FINDER_MIN_EXPECTED_DISPLACEMENT)
		&& (sFCU.sFlightControl.sBlender.sDisplacement_m.sfromLaser.u32Value < LASER_RANGE_FINDER_MAX_EXPECTED_DISPLACEMENT))
		//if laser range finder value is in laser range finder first expected range,
		{
			sFCU.sFlightControl.sBlender.sWorking.u8LaserRangeFinder_Valid = 1U;
			//set the laser range finder as valid.
			sFCU.sFlightControl.sBlender.sWorking.u32StarttransitionTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
			// store the time
		}
	}

	if(sFCU.sFlightControl.sBlender.sWorking.u8LaserRangeFinder_Valid < 1U)
	// If laser sensor is invalid
	{
		// set front x position to front x position from contrast sensors
		sFCU.sFlightControl.sBlender.sWorking.s32Disp_mm = sFCU.sFlightControl.sBlender.sDisplacement_m.sfromContrast.u32Value;
	}
	else
	{
		Luint32 u32CurrentTime;
		u32CurrentTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter - sFCU.sFlightControl.sBlender.sWorking.u32StarttransitionTime;

		if (u32CurrentTime < TRANSITION_DELAY_FROM_CONTRAST_TO_LASER_SENSOR)
		// if have passed a time lower than the transition time from the moment
		// when the laser distance sensor measuremen become valid
		{
			Lfloat32 f32Coef;
			Luint32 u32TmpLaserDisp, u32TmpContrDisp;

			// transition delay from contrast sensors to laser range finder
			f32Coef = u32CurrentTime / TRANSITION_DELAY_FROM_CONTRAST_TO_LASER_SENSOR;
			u32TmpContrDisp = (Luint32)(sFCU.sFlightControl.sBlender.sDisplacement_m.sfromContrast.u32Value * (1 - f32Coef));
			u32TmpLaserDisp = (Luint32)(sFCU.sFlightControl.sBlender.sDisplacement_m.sfromLaser.u32Value * f32Coef);
			// compute the unfiltered displacement as combination of the two sensors
			// on the result vill be applied the filtering
			sFCU.sFlightControl.sBlender.sWorking.s32Disp_mm = u32TmpContrDisp + u32TmpLaserDisp;
		}
		else
		// the laser distance sensor measuremen is valid from a time greater than the transition time
		{
			// set front x position to front x position from laser sensors
			sFCU.sFlightControl.sBlender.sWorking.s32Disp_mm = sFCU.sFlightControl.sBlender.sDisplacement_m.sfromLaser.u32Value;
		}
	}
}






//update the velocity from the accelometer subystems
void vFCU_FCTL_BLENDER__Veloc_UpdateFrom_Accel(Luint8 u8Channel, Luint32 u32Veloc_mms)
{
	sFCU.sFlightControl.sBlender.sVelocity_ms.sfromAccel.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sVelocity_ms.sfromAccel.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}

//update the velocity from the LRF
void vFCU_FCTL_BLENDER__Veloc_UpdateFrom_LRF(Luint8 u8Channel, Luint32 u32Veloc_mms)
{
	sFCU.sFlightControl.sBlender.sVelocity_ms.sfromLaser.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sVelocity_ms.sfromLaser.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}

//Update the velocity from the contrast sensor system
void vFCU_FCTL_BLENDER__Veloc_UpdateFrom_Contrast(Luint8 u8Channel, Luint32 u32Veloc_mms)
{
	sFCU.sFlightControl.sBlender.sVelocity_ms.sfromContrast.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sVelocity_ms.sfromContrast.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}

//update the Acceleration from the accelometer subystems
void vFCU_FCTL_BLENDER__Accel_UpdateFrom_Accel(Luint8 u8Channel, Luint32 u32Accel_mmss)
{
	sFCU.sFlightControl.sBlender.sAccelleration_mss.sfromAccel.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sAccelleration_mss.sfromAccel.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}

//update the Acceleration from the LRF
void vFCU_FCTL_BLENDER__Accel_UpdateFrom_LRF(Luint8 u8Channel, Luint32 u32Accel_mmss)
{
	sFCU.sFlightControl.sBlender.sAccelleration_mss.sfromLaser.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sAccelleration_mss.sfromLaser.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}

//Update the Acceleration from the contrast sensor system
void vFCU_FCTL_BLENDER__Accel_UpdateFrom_Contrast(Luint8 u8Channel, Luint32 u32Accel_mmss)
{
	sFCU.sFlightControl.sBlender.sAccelleration_mss.sfromContrast.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sAccelleration_mss.sfromContrast.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}


//update the Displacement from the accelometer subystems
void vFCU_FCTL_BLENDER__Displacement_UpdateFrom_Accel(Luint8 u8Channel, Luint32 u32Disp_mm)
{
	sFCU.sFlightControl.sBlender.sDisplacement_m.sfromAccel.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sDisplacement_m.sfromAccel.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}

//update the Displacement from the LRF
//Note: LRF may be distance remaining, need to blend properly
void vFCU_FCTL_BLENDER__Displacement_UpdateFrom_LRF(Luint8 u8Channel, Luint32 u32Disp_mm)
{
	sFCU.sFlightControl.sBlender.sDisplacement_m.sfromLaser.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sDisplacement_m.sfromLaser.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}

//Update the Displacement from the contrast sensor system
//Note this may be track elapsed position
void vFCU_FCTL_BLENDER__Displacement_UpdateFrom_Contrast(Luint8 u8Channel, Luint32 u32Disp_mm)
{
	sFCU.sFlightControl.sBlender.sDisplacement_m.sfromContrast.u32Value = u32Disp_mm;
	sFCU.sFlightControl.sBlender.sDisplacement_m.sfromContrast.u32UpdateTime = sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter;
}


/***************************************************************************//**
 * @brief
 * 100ms Timer interrupt.
 */
void vFCU_LASERDIST__100MS_ISR(void)
{
		sFCU.sFlightControl.sBlender.sWorking.u32TimeCounter++;
}




#endif //C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL
#ifndef C_LOCALDEF__LCCM655__ENABLE_FLIGHT_CONTROL
	#error
#endif

#endif //#if C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE == 1U
//safetys
#ifndef C_LOCALDEF__LCCM655__ENABLE_THIS_MODULE
	#error
#endif
