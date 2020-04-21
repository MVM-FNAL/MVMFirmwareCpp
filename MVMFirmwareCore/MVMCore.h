// MVMCore.h

#ifndef _MVMCORE_h
#define _MVMCORE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include "HAL.h"
#include "MVM_StateMachine.h"
#include "Alarms.h"
#include "TidalVolume.h"
#include "ConfigManager.h"

class MVMCore
{
public:
	void Init();
	void Tick();
	bool DataAvailableOnUART0();
	String ReadUART0UntilEOL();
	bool WriteUART0(String s);
	bool SetParameter(String p, String v);
	String GetParameter(String p);
	void ZeroSensors(float *sensors, int *count);
	bool FlushPipes(bool run);
	void CalibrateOxygenSensor();
	
private:
	
	HAL MVM_HAL;
	ConfigManagerClass CMC;
	t_SystemStatus sys_s;
	AlarmClass Alarms;
	TidalVolumeClass TidalVolumeExt;

	CircularBuffer  *MEM_Ppatient_LP;
	MVM_StateMachine MVM_SM;
	
	float old_delta_ppatient;
	uint64_t last_respiratory_act;
	float averaged_PPatient;

	void PPatient_Event();
	void FlowIn_Event();
	void PLoop_Event();
	void FlowVenturi_Event();


	void NewCycle_Event();
	void Exhale_Event();
	void EndCycle_Event();
	void ConfigurationChanged_Event();
	void HardwareAlarm_Event(t_ALARM alarm_code);
};
#endif

