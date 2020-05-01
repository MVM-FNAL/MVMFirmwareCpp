// driver_SFM3000.h

#ifndef _DRIVER_SFM3000_h
#define _DRIVER_SFM3000_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "DriverContext.h"

class SensorSFM3000
{
public :
	bool Init(t_i2cdevices device, void* hw_handle);
	bool doMeasure(float* Flow, float* T);
	float GetIntegral();
	void ResetIntegral();

private:
	t_i2cdevices i2c_device;
	uint8_t crc8(const uint8_t data, uint8_t crc);
	bool MeasureFluxRaw(uint32_t* raw);

	HW* hwi;
	DebugIfaceClass* dbg;

	float Integral;
	bool _initialized = false;
};

#endif

