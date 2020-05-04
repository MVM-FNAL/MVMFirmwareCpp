// 
// 
// 

#include "TidalVolume.h"
#include <math.h>
#define VOL_COMP 0.4

/**
 * \brief	Initialize the Class.
 * 
 * \param hal	Pointer to MVM HAL class
 */
void TidalVolumeClass::Init(HAL* hal)
{
	TotalVolume=0;
	InspVolumeSensirion=0;
	InspVolumeVenturi=0;
	ExpVolumeVenturi=0;
	TidalCorrection = 1;
	LastSensirionVolume = 0;
	LastVenturiVolume = 0;
	FLUX=0;
	Status = 0;
	_HAL = hal;
}


/**
 *	Meaning of the Status variable
 * 
 *		Status =	0		NO INTEGRATION
 *					1		INHALE
 *					2		EXHALE
 * 
 * 
 *	Status is driven by the three events exported by the class
 *		-DoNewCycle		Triggered at the beginning of a new respiratory cycle
 *		-DoExhale		Triggered at the beginning of expiratory phase
 *		-DoEndCycle		Triggered at the end of the cycle
 * .
 */

/**
 * \brief This function is called every time we have a new data from Sensirion.
 * 
 * The function accumulate date only in inhale phase 
 * 
 * \param flux_sens		flux measured by Sensirion in slpm
 */
void TidalVolumeClass::PushDataSens(float flux_sens)
{
	
	//Calculate deltaT for the volume in seconds
	float dT;
	dT = ((float)_HAL->Get_dT_millis(last_meas_t))/1000.0;
	last_meas_t = _HAL->GetMillis();

	
	if (Status==1)	//Inhale
	{ 
		//Volume calculated from the flow (using Sensirion)
		TotalVolume += flux_sens* dT / 60.0;					
		//Ispired volume from the flow using Sensirion
		InspVolumeSensirion += flux_sens* dT/60.0;
		//Flow
		FLUX = flux_sens;
		//Measure of max flow
		FluxMax = flux_sens > FluxMax ? flux_sens : FluxMax;
		liveFlux = FLUX;
		liveVolume = TotalVolume;
	}
	else
	{
		if (Status == 2) //Hexale
		{
			//Do nothing in expiration, whe have no data

		}
		else
		{
			liveFlux = flux_sens;
		}
	}
}

/**
 * \brief This function is called every time we have a new data from Venturi.
 * 
 * \param flux_venturi		flux measured by Venturi in slpm
 */
void TidalVolumeClass::PushDataVenturi(float flux_venturi)
{
	//Calculate deltaT for the volume in seconds
	float dT;
	dT = ((float)_HAL->Get_dT_millis(last_meas_t_b)) / 1000.0;
	last_meas_t_b = _HAL->GetMillis();

	if (Status == 1) //Inhale
	{
		//Integrate volume of venturi. It will used to calculate compensation
		//coefficient and to detect Venturi inversion
		InspVolumeVenturi += flux_venturi * dT / 60.0;
	}
	else
	{
		if (Status == 2) //Hexale
		{
			//Calculate volume of venturi. It is realistic to suppose than
			//in exhale phase the flux_venturi will have negative flow
			//and integral will trend to zero

			//Calculate the exhale flow. It will be used for venturi inversion
			// or not connect alarm
			ExpVolumeVenturi += flux_venturi * dT / 60.0;

			//Use the TadalCorrection variable to calculate the TotalVolume
			//using the measure from sensirion and inhale venturi to correct
			//the inexact measure of venturi flow
			if (TidalCorrection > 0) {
				float vf_clamp=0;
				//Clamp to zero the Venturi flow if smaller that 1slpm
				//This is an hack but it is necessary because the venturi
				//never measure 0 at 0 flow. It always has a small bias
				//that make integral diverge
				vf_clamp = fabs(flux_venturi) > 1 ? flux_venturi : 0;
				TotalVolume += (vf_clamp *  TidalCorrection) * dT / 60.0;
				FLUX = flux_venturi  * TidalCorrection;
				liveFlux = FLUX;
				liveVolume = TotalVolume;
			}
		}
		else
		{
			
			//do nothing
		}
	}
}

/**
 * \brief Event handler called at every new respiratory cycle.
 * 
 * The event resets all integrals. We must reset volume integral
 * due to bias in venturi measure that make integral diverging
 */
void TidalVolumeClass::DoNewCycle()
{
	TotalVolume = 0;
	InspVolumeSensirion = 0;
	InspVolumeVenturi = 0;
	ExpVolumeVenturi = 0;
	TidalCorrection = 1;
	FLUX = 0;
	Status = 1;
	FluxMax = 0;
	last_meas_t = _HAL->GetMillis();
	last_meas_t_b = _HAL->GetMillis();
}

/**
 * \brief	Event handler called at every passing from inhale to exhale.
 * 
 * The event store the current inhale volume for both Sensirion and venturi
 * and calculate the ratio between venturi and Sensirion during the
 * inspiratory phase to compensate the venturi in expiratory phase
 */
void TidalVolumeClass::DoExhale()
{
	LastVenturiVolume = InspVolumeVenturi;
	LastSensirionVolume = InspVolumeSensirion;
	currentTvIsnp = InspVolumeSensirion;
	currentFluxPeak = FluxMax;
	
	if (InspVolumeSensirion > 0)
		TidalCorrection = InspVolumeVenturi / InspVolumeSensirion;
	else
		TidalCorrection = 1;



	
	//Serial.println("SENS: " + String(InspVolumeVenturi) + " VENT: " + String(InspVolumeSensirion) + " COR: " + String(TidalCorrection));
	Status = 2;
}

/**
 * \brief	Event handler called at every end of cycle.
 * 
 * Calculate the espiratory volume using the TidalCorerction and 
 * inverting the sign. (present it as positive number)
 * 
 */
void TidalVolumeClass::DoEndCycle()
{
	if (TidalCorrection > 0) {
		currentTvEsp = -1.0 * ExpVolumeVenturi * TidalCorrection;
	}
	Status = 0;
}

//                  #     # ### 
//                  ##    #  #  
//                  # #   #  #  
//                  #  #  #  #  
//                  #   # #  #  
//                  #    ##  #  
//                  #     # ### 
//
// Nuclear Instruments 2020 - All rights reserved
// Any commercial use of this code is forbidden
// Contact info@nuclearinstruments.eu
