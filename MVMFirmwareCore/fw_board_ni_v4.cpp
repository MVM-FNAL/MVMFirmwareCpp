// 
// 
// 

#include "fw_board_ni_v4.h"


#include <Wire.h>


#define VALVE_IN_PIN		A1 
#define VALVE_OUT_PIN		32 

#define BUZZER				21
#define ALARM_LED			17
#define ALARM_RELE			A12
#define BREETHE				4

#define TCAADDR 0x70

/**
 *  \brief	Initialize hardware peripheral in the system
 * 
 *  The function configure:
 *		- Serial Port
 *		- PWM
 *		- PINS
 *		- I2C
 *		- Initialize the device list iic_devs
 *		- Variables
 * 
 *	It also perform a I2C scan
 * 
 * \return 
 */
bool HW_V4::Init()
{

	//Init serial port 115200,8,n,1
	Serial.begin(115200);

	//Init PWM for PV1, 10KHz, 12 bit
	ledcSetup(0, 10000, 12);
	ledcAttachPin(VALVE_IN_PIN, 0);
	ledcWrite(0, 0);

	//Init pins: PV2, ALARM LED, BUZZER, RELE
	digitalWrite(VALVE_OUT_PIN, LOW);
	digitalWrite(ALARM_LED, LOW);
	digitalWrite(BUZZER, LOW);
	digitalWrite(ALARM_RELE, HIGH);

	pinMode(VALVE_OUT_PIN, OUTPUT);
	pinMode(ALARM_LED, OUTPUT);
	pinMode(BUZZER, OUTPUT);
	pinMode(ALARM_RELE, OUTPUT);

	//Start I2C Master and Scan bus
	Wire.begin();

	for (int i = 0; i < 8; i++) {
		i2c_MuxSelect(i);
		Serial.println("SCAN I2C BUS: " + String(i));
		__service_i2c_detect();
	}

	//Init list of devices on I2C bus

	iic_devs[0].t_device = IIC_PS_0;			//NAME OF THE SENSORE
	iic_devs[0].muxport = 0;					//Multiplexer PORT
	iic_devs[0].address = 0x76;					//I2C ADDRESS

	iic_devs[1].t_device = IIC_PS_1;
	iic_devs[1].muxport = 0;
	iic_devs[1].address = 0x77;

	iic_devs[2].t_device = IIC_PS_2;
	iic_devs[2].muxport = 1;
	iic_devs[2].address = 0x76;

#ifdef USE_SPIROMETER_SFM3019
	iic_devs[3].t_device = IIC_FLOW1;
	iic_devs[3].muxport = 1;
	iic_devs[3].address = 0x2E;

#endif

#ifdef USE_SPIROMETER_SFM3000
	iic_devs[3].t_device = IIC_FLOW1;
	iic_devs[3].muxport = 1;
	iic_devs[3].address = 0x40;
#endif

	iic_devs[4].t_device = IIC_ADC_0;
	iic_devs[4].muxport = 4;
	iic_devs[4].address = 0x48;

	iic_devs[5].t_device = IIC_SUPERVISOR;
	iic_devs[5].muxport = 3;
	iic_devs[5].address = 0x22;


	iic_devs[6].t_device = IIC_MUX;
	iic_devs[6].muxport = -1;
	iic_devs[6].address = 0x70;

	iic_devs[7].t_device = IIC_GENERAL_CALL_SENSIRION;
	iic_devs[7].muxport = 1;
	iic_devs[7].address = 0x00;


	batteryStatus_reading_LT = GetMillis();


	currentBatteryCharge = 100;
	pWall=true;
	pIN=3;
	BoardTemperature=25;
	
	//init supervisor watchdog

	/*
	* During development we keep disabled supervisor watchdog
	* otherwise it will trigger low level alarm every time 
	* we reprogram ESP
	* 
	* 0 : DEVELOPMENT	- NO SUPERVISOR WATCHDOG
	* 1 : PRODUCTION	- ENABLE SUPERVISOR WATCHDOG
	*/

	
}

/**
 * \brief	Write buffer to I2C bus
 * 
 * This function also switch mux port in order to access to required device
 * 
 * \param device		name of the device listened in the t_i2cdevices iic_dev 
 * \param wbuffer		buffer to be written
 * \param wlength		length of the buffer
 * \param stop			add stop bit at end of transaction
 * \return				true if success
 */
bool HW_V4::I2CWrite(t_i2cdevices device, uint8_t* wbuffer, int wlength, bool stop)
{
	uint8_t address;
	uint8_t result;
	
	//Search I2C device by name
	t_i2cdev dev = GetIICDevice(device);
	address = dev.address;

	//Switch multiplexer
	i2c_MuxSelect(dev.muxport);

	//Arduino I2C Write operation
	Wire.beginTransmission(address);
	for (int i = 0;i < wlength; i++)
		Wire.write(wbuffer[i]);
	result = Wire.endTransmission();

	if (result != 0)
		return false;
	else
		return true;
}

/**
 * \brief	Write buffer to I2C bus and read data
 * 
 * This function also switch mux port in order to access to required device
 * 
 * \param device		name of the device listened in the iic_dev (t_i2cdevices)
 * \param wbuffer		buffer to be written
 * \param wlength		length of the buffer to be written
 * \param rbuffer		reading buffer (should be allocated externally)
 * \param rlength		number of bytes to be read
 * \param stop			add stop at end of transaction
 * \return				true if success
 */
bool HW_V4::I2CRead(t_i2cdevices device, uint8_t* wbuffer, int wlength, uint8_t* rbuffer, int rlength, bool stop)
{
	uint8_t address;
	uint8_t count;
	uint8_t result;

	//Search I2C device by name
	t_i2cdev dev = GetIICDevice(device);
	address = dev.address;

	//Switch multiplexer
	i2c_MuxSelect(dev.muxport);

	//Arduino I2C Write operation
	Wire.beginTransmission(address);
	for (int i = 0;i < wlength; i++)
		Wire.write(wbuffer[i]);
	result = Wire.endTransmission();
	if (result != 0)
		return false;

	//Arduino I2C Read operation
	count = Wire.requestFrom((uint16_t)address, (uint8_t)rlength, stop);
	if (count < rlength)
		return false;

	for (int i = 0;i < rlength; i++)
	{
		rbuffer[i] = Wire.read();
	}

	return true;
}

/**
 * \brief	Read I2c data
 * 
 * This function also switch mux port in order to access to required device
 * 
 * \param device		name of the device listened in the iic_dev (t_i2cdevices)
 * \param rbuffer		reading buffer (should be allocated externally)
 * \param rlength		number of bytes to be read
 * \param stop			add stop at end of transaction
 * \return				true if success
 */
bool HW_V4::I2CRead(t_i2cdevices device, uint8_t* rbuffer, int rlength, bool stop)
{
	uint8_t count;
	uint8_t address;
	t_i2cdev dev = GetIICDevice(device);
	address = dev.address;
	i2c_MuxSelect(dev.muxport);

	count = Wire.requestFrom((uint16_t)address, (uint8_t)rlength, stop);

	if (count < rlength)
		return false;


	for (int i = 0;i < rlength; i++)
	{
		rbuffer[i] = Wire.read();
	}

	return true;
}

/**
 * \brief	Control PWM device (PV1)
 * 
 * \param id		name of the device (enum hw_pwm)
 * \param value		value from 0 to 100
 * \return			always true
 */
bool HW_V4::PWMSet(hw_pwm id, float value)
{

	if ((value < 0) || (value > 100.0)) return false;

	switch (id)
	{
	case PWM_PV1:
		uint32_t v = (uint32_t)value * 4095.0 / 100.0;
		ledcWrite(0, v);
		/*
		* The breethe signal is used by the supervisor to check
		* whenever the driver for PV1 and PV2 is not working
		* We need to commutate breethe synchronous with PV1
		*/
		if (v > 0)
			digitalWrite(BREETHE, HIGH);
		break;

	}


	return true;
}

/**
 * \brief	Set a GPIO Status (IE Control PV6, Alarms, etc)
 * 
 * \param id		name of the GPIO (enum hw_gpio)
 * \param value		true: logic level HI, low: logic level LOW
 * \return			true if GPIO exists
 */
bool HW_V4::IOSet(hw_gpio id, bool value)
{
	switch (id)
	{
	case GPIO_PV2:
		digitalWrite(VALVE_OUT_PIN, value ? HIGH : LOW);
		/*
		* The breethe signal is used by the supervisor to check
		* whenever the driver for PV1 and PV2 is not working
		* We need to commutate breethe synchronous with PV2
		*/
		if (value==LOW)
			digitalWrite(BREETHE, LOW);
		break;
	case GPIO_BUZZER:
		digitalWrite(BUZZER, value ? HIGH : LOW);
		break;
	case GPIO_LED:
		digitalWrite(ALARM_LED, value ? HIGH : LOW);
		break;
	case GPIO_RELEALLARM:
		digitalWrite(ALARM_RELE, value ? HIGH : LOW);
		break;
	default:
		return false;
		break;
	}
	return true;
}

/**
 * \brief	Get GPIO Status
 * 
 * \param id		name of the GPIO (enum hw_gpio)
 * \param value		OUTPUT true: logic level HI, low: logic level LOW
 * \return			true if GPIO exists
 */
bool HW_V4::IOGet(hw_gpio id, bool* value)
{
	switch (id)
	{
	case GPIO_PV2:
		*value = digitalRead(VALVE_OUT_PIN);
		break;
	case GPIO_BUZZER:
		*value = digitalRead(BUZZER);
		break;
	case GPIO_LED:
		*value = digitalRead(ALARM_LED);
		break;
	case GPIO_RELEALLARM:
		*value = digitalRead(ALARM_RELE);
		break;
	default:
		return false;
		break;
	}
	return true;
}

/**
 * \brief	Implements a blocking delay
 * 
 * \param ms		delay in ms
 */
void HW_V4::__delay_blocking_ms(uint32_t ms)
{
	delay(ms);
}

/**
 * \brief	Print a message on console used for Debug
 * 
 * For ESP32 this is the same console of interface bus
 * this is dangerous and in production DEBUG LEVEL should be set
 * in order to be sure that no debug message is forwarded on
 * this console otherwise GUI will crash
 * 
 * \param s		String to be print
 */
void HW_V4::PrintDebugConsole(String s)
{
	Serial.print(s);
}


/**
 * \brief	Print a message on console used for Debug with a CR+LR at end
 *
 * For ESP32 this is the same console of interface bus
 * this is dangerous and in production DEBUG LEVEL should be set
 * in order to be sure that no debug message is forwarded on
 * this console otherwise GUI will crash
 *
 * \param s		String to be print
 */
void HW_V4::PrintLineDebugConsole(String s)
{
	Serial.println(s);
}

/**
 * \brief	Tick function must be called periodically 
 * 
 * The function performs in this driver polling task
 * 
 * Every seconds the function will contact the supervisor to do:
 *	- read battery charge
 *	- read power status (power line/battery)
 *  - read input pressure
 *	- read board temperature
 *	- read supervisor alarm flags
 *	- reset supervisor watchdog
 * 
 */
void HW_V4::Tick()
{
	if (Get_dT_millis(batteryStatus_reading_LT)>1000)
	{
		batteryStatus_reading_LT = GetMillis();
		currentBatteryCharge = (float)ReadSupervisor(0x51);
		currentBatteryCharge = currentBatteryCharge < 0 ? 0 : currentBatteryCharge;
		currentBatteryCharge = currentBatteryCharge > 100 ? 100 : currentBatteryCharge;
		pWall = ReadSupervisor(0x52) >0 ? false : true ;
		pIN = ((float)ReadSupervisor(0x50));
		BoardTemperature = ((float)ReadSupervisor(0x56)/10.0);
		HW_AlarmsFlags = (uint16_t)ReadSupervisor(0x57);

		//reset supervisor watchdog
 		WriteSupervisor(0x00, 1);

		if (EnableWatchdogSupervisor){
			WriteSupervisor(0x01, 1);
			EnableWatchdogSupervisor = false;
			}
		//Serial.println("Battery: " + String(currentBatteryCharge) + " PWALL: " + String (pWall));
	}
	

	return;
}


/**
 * \brief	API to read battery charge and power supply status
 * 
 * \param batteryPowered	true the system is battery powered
 *							false the system is powered from 220v
 * \param charge			% of battery charge
 */
void HW_V4::GetPowerStatus(bool* batteryPowered, float* charge)
{
	*batteryPowered = pWall ? false:true;
	*charge = currentBatteryCharge ;

}


/**
 * \brief	API to read if on the communication interfaces there are bytes
 *			to be read
 * 
 * \return	true if bytes are available
 */
bool HW_V4::DataAvailableOnUART0()
{
	return Serial.available();
}

/**
 * \brief	API to write a string to the communication interfaces
 * 
 * \param s		String to be written
 * \return		always true
 */
bool HW_V4::WriteUART0(String s)
{
	Serial.println(s);
	return true;
}

/**
 * \brief	Return a \n terminated string from communication interface
 * 
 * \return		Recevied string
 */
String HW_V4::ReadUART0UntilEOL()
{
	//PERICOLO. SE IL \n NON VIENE INVIATO TUTTO STALLA!!!!
	return Serial.readStringUntil('\n');
}

/**
 * \brief	Retrun milliseconds from system startup
 * 
 * \return  milliseconds from boot
 */
uint64_t HW_V4::GetMillis()
{
	return (uint64_t)millis();
}

/**
 * \brief	Calculate delta time in ms respect now
 * 
 * \param ms	start ms
 * \return		delta ms
 */
int64_t HW_V4::Get_dT_millis(uint64_t ms)
{
	return (int64_t)(millis() - ms);
}


/**
 * \brief Detect all I2C devices on bus
 * 
 */
void HW_V4::__service_i2c_detect()
{
	byte error, address;
	int nDevices;
	Serial.println("Scanning... I2C");
	nDevices = 0;
	for (address = 1; address < 127; address++) {
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			if (address < 16) {
				Serial.print("0");
			}
			Serial.println(address, HEX);
			nDevices++;
		}
		else if (error == 4) {
			Serial.print("Unknow error at address 0x");
			if (address < 16) {
				Serial.print("0");
			}
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0) {
		Serial.println("No I2C devices found\n");
	}
	else {
		Serial.println("done\n");
	}
}

/**
 * \brief	Switch I2C multiplexer
 * 
 * Check if mux is already in the correct position otherwise commutate
 * 
 * \param i
 */
void HW_V4::i2c_MuxSelect(uint8_t i)
{
	if (i > 7)
		return;

	if (i < 0)
		return;

	if (current_muxpos == i) return;

	current_muxpos = i;

	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << i);
	Wire.endTransmission();
	delayMicroseconds(500);
}

/**
 * \brief	Search in the iic_devs list a particular device and return descriptor
 * 
 * \param device	device (t_i2cdevices) to be searched for
 * \return			device handler
 */
t_i2cdev HW_V4::GetIICDevice(t_i2cdevices device)
{
	for (int i = 0; i < IIC_COUNT; i++)
	{
		if (iic_devs[i].t_device == device)
		{
			return iic_devs[i];
		}
	}
}

/**
 * \brief	Read a supervisor register
 * 
 * \param i_address		address of the register in the supervisor register file
 * \return				value of the register
 */
uint16_t HW_V4::ReadSupervisor(uint8_t i_address)
{
	uint8_t wbuffer[4];
	uint8_t rbuffer[4];
		
	wbuffer[0] = i_address;
	I2CRead(IIC_SUPERVISOR, wbuffer, 1, rbuffer, 2, true);

	uint16_t a;

	a = (rbuffer [1]<< 8) | rbuffer[0];
	return a;
}

/**
 * \brief	Write a supervisor register
 * 
 * \param i_address		address of the register in the supervisor register file
 * \param write_data	value of the register
 */
void HW_V4::WriteSupervisor( uint8_t i_address, uint16_t write_data)
{
	uint8_t wbuffer[4];
	wbuffer[0] = i_address;
	wbuffer[1] = write_data & 0xFF;
	wbuffer[2] = (write_data >> 8) & 0xFF;

	I2CWrite(IIC_SUPERVISOR, wbuffer, 3, true);

}

/**
 * \brief	API return pressure on input of MVM
 * 
 * \return		pressure in mbar
 */
float HW_V4::GetPIN()
{
	return pIN;
}

/**
 * \brief	API return temperature of the board
 * 
 * \return		temperature in °C
 */
float HW_V4::GetBoardTemperature()
{
	return BoardTemperature;
}

/**
 * \brief	API return Supervisor managed alarms
 * 
 *			bit 0 : input pressure too low
 *			bit 1 : input pressure too high
 *			bit 2 : board temperature too low
 *			bit 3 : board temperature too high
 *			bit 4 : no activity on the valves
 *			bit 5 : power regulator issue
 *			bit 6 : battery charge critical
 * 
 * 
 * \return	encoded alarm status
 */
uint16_t HW_V4::GetSupervisorAlarms()
{
	return HW_AlarmsFlags;
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
