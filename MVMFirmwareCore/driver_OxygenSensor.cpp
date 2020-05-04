// 
// 
// 

#include "driver_OxygenSensor.h"


/**
 * \brief	Initialize Oxygen sensor class.
 * 
 * \param model		Model of the oxygen sensor
 * \param handle	handle to the DriverContext
 */
void OxygenSensor::Init(t_OxygenSensorModel model, void* handle)
{
	DriverContext* dc;
	dc = (DriverContext*)handle;
	hwi = (HW*)dc->hwi;
	dbg = (DebugIfaceClass*)dc->dbg;

	_oxmodel = model;

	//Load the calibration coefficient in fucntion of the model of
	//oxygen sensor
	if (_oxmodel == OxygenSensorA)
	{
		calib_q = 0.00452689;
		calib_m = -1.63;
		calib_interval_seconds = 3600 * 24 * 7;		//7 days
	}
	
	//Reset varaible
	_adc_oxygen=0;
	_temperature=0;
	calib_second_counter=0;
	second_counter=0;

	second_counter = hwi->GetMillis();
}

/**
 * \brief	Return the oxygen concentration.
 * 
 * \return	Oxygen in %
 */
float OxygenSensor::GetConcentration()
{
	float o2;
	o2 = _adc_oxygen * calib_q + calib_m;
	o2 = o2 > 100 ? 100 : o2;
	return o2;

}

/**
 * \brief	Event triggered when the ADC converted the analog value.
 * 
 * \param adc_oxygen	analog value read by ADC
 * \param temprature	gas temperature
 * \return 
 */
bool OxygenSensor::setData(float adc_oxygen, float temprature)
{
	_adc_oxygen = adc_oxygen;
	_temperature = temprature;
}

/**
 * \brief	Calibrate the sesor at 21%.
 * 
 */
void OxygenSensor::CalibrateAir()
{
	calib_m = -((_adc_oxygen*calib_q) - 21);
	calib_second_counter = 0;
}


/**
 * \brief	Calibrate the sesor at 100%.
 *
 */
void OxygenSensor::CalibratePureOxygen()
{
	calib_m = -((_adc_oxygen*calib_q) - 100);
	calib_second_counter = 0;
}

/**
 * \brief	Check if sensor must be recalibrated.
 * 
 * This function check it time from last calibration is higher
 * than the calibration period of selected sensor
 * 
 * \return true if must be recalibrated
 */
bool OxygenSensor::CheckNeedRecalibrate()
{
	if (calib_second_counter > calib_interval_seconds)
		return true;
	else
		return false;
}

/**
 * \brief	Period action in the driver.
 * 
 */
void OxygenSensor::Tick()
{
	if (hwi->Get_dT_millis(second_counter) > 1000)
	{
		calib_second_counter++;
	}
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
