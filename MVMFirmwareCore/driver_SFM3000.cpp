// 
// 
// 

#include "driver_SFM3000.h"


bool SensorSFM3000::Init(t_i2cdevices device, void* hw_handle)
{

	DriverContext* dc;
	dc = (DriverContext*)hw_handle;
	hwi = (HW*)dc->hwi;
	dbg = (DebugIfaceClass*)dc->dbg;
	i2c_device = device;

	uint8_t wBuffer[6];
	wBuffer[0] = 0x10;
	wBuffer[1] = 0x00;

	hwi->I2CWrite(i2c_device, wBuffer, 2, true);

	return hwi->I2CWrite(i2c_device, wBuffer, 2, true);

	
}

bool SensorSFM3000::doMeasure(float* Flow, float* T)
{
	uint32_t result = 0;
	bool bres = MeasureFluxRaw(&result);
	if (!bres) return false;

	int offset = 32000; // Offset for the sensor
	float scale = 140.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8
	float _flow = ((float)result - offset) / scale;
	*Flow = _flow;
	*T = 24.0;
	Integral += _flow;

	return true;

}
float SensorSFM3000::GetIntegral()
{
	return Integral;
}
void SensorSFM3000::ResetIntegral()
{
	Integral = 0;
}


uint8_t SensorSFM3000::crc8(const uint8_t data, uint8_t crc)
{
	crc ^= data;
	for (uint8_t i = 8; i; --i) {
		crc = (crc & 0x80)
			? (crc << 1) ^ 0x31
			: (crc << 1);
	}
	return crc;
}

bool SensorSFM3000::MeasureFluxRaw(uint32_t *raw)
{
	uint8_t rBuffer[6];
	bool bres = hwi->I2CRead(i2c_device, rBuffer, 3, true);
	if (!bres) return false;

	uint16_t a = rBuffer[0]; // first received byte stored here. The variable "uint16_t" can hold 2 bytes, this will be relevant later
	uint8_t b = rBuffer[1]; // second received byte stored here
	uint8_t crc = rBuffer[2]; // crc value stored here
	uint8_t mycrc = 0xFF; // initialize crc variable

	a = (a << 8) | b; // combine the two received bytes to a 16bit integer value

	uint32_t Flow = a;

	*raw = Flow;
	
	return true;
}