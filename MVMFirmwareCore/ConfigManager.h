// ConfigManager.h

#ifndef _CONFIGMANAGER_h
#define _CONFIGMANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "Configuration.h"
#include "SystemStatus.h"
#include "Alarms.h"

/**
 * \brief	Configuration class. Store config parameters and manage communicate interface
 * 
 * The class store all configuration and allows the interoperability with the GUI
 * or control scripts
 * It does not directly manage the communication. It just export the SetParameter and 
 * GetParameter function
 * 
 * The system configuration is stored in the core_config t_config struct.
 * 
 * The class initialize configuration with default parameters
 */
class ConfigManagerClass 
{
 protected:

	 

 public:
	 bool SetParameter(String p, String v);
	 String GetParameter(String p);
	 void Init(void *_core, t_SystemStatus* _sys_s, AlarmClass *_Alarms);
	 t_config core_config;

	 std::function<void()> callback_BeforeConfigurationGet = NULL;
	 std::function<void()> callback_BeforeConfigurationSet = NULL;
	 std::function<void()> callback_AfterConfigurationSet = NULL;

	 void addHandler_BeforeConfigurationGet(std::function<void()> callback)
	 {
		 callback_BeforeConfigurationGet = callback;
	 }

	 void addHandler_BeforeConfigurationSet(std::function<void()> callback)
	 {
		 callback_BeforeConfigurationSet = callback;
	 }

	 void addHandler_AfterConfigurationSet(std::function<void()> callback)
	 {
		 callback_AfterConfigurationSet = callback;
	 }

private:
	AlarmClass *Alarms;
	void* core;
	t_SystemStatus *sys_s;

	uint32_t GenerateFlag(int alarm_code);

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
