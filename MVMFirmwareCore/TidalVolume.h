// TidalVolume.h

#ifndef _TIDALVOLUME_h
#define _TIDALVOLUME_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "HAL.h"

/**
 * \brief This class implement the calculation of flux and Volume.
 * 
 * The class is triggered by the availability of date from the two spirometers
 * and calculate:
 *		- Ispiratory volume
 *		- Expiratory volume
 *		- Total Volume
 *		- Flux
 *
 * The class use the Sensirion measure to compensate the venturi measure
 * in order to make more accurate the venturi
 * 
 * It is also used to generate alarm on venturi inversion
 */
class TidalVolumeClass
{
private:
	HAL* _HAL;
	float TotalVolume=0;
	float InspVolumeSensirion=0;
	float InspVolumeVenturi=0;
	float ExpVolumeVenturi=0;
	float TidalCorrection=1;
	float FLUX=0;
	int Status=0;
	uint64_t last_meas_t;
	uint64_t last_meas_t_b;
	float FluxMax=0;

public:
	float liveFlux;
	float liveVolume;
	float currentTvEsp;
	float currentTvIsnp;
	float currentFluxPeak;
	float LastSensirionVolume;
	float LastVenturiVolume;
	void Init(HAL* hal);
	void PushDataSens(float flux_sens);
	void PushDataVenturi(float flux_venturi);
	void DoNewCycle();
	void DoExhale();
	void DoEndCycle();
};

#endif

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
