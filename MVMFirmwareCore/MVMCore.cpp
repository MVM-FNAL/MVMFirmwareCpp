// 
// 
// 



#include "MVMCore.h"
#include <functional>

void MVMCore::Init()
{
	MVM_HAL.Init();
	
	CMC.Init(&sys_s);

	MVM_HAL.SetInputValve(0);
	MVM_HAL.SetOutputValve(0);
	MVM_HAL.SetAlarmLed(false);
	MVM_HAL.SetAlarmRele(false);
	MVM_HAL.SetBuzzer(false);
	MVM_HAL.addHandler_PLoop(std::bind(&MVMCore::PLoop_Event, this));
	MVM_HAL.addHandler_PPatient(std::bind(&MVMCore::PPatient_Event, this));
	MVM_HAL.addHandler_FlowSens(std::bind(&MVMCore::FlowIn_Event, this));
	MVM_HAL.addHandler_FlowVenturi(std::bind(&MVMCore::FlowVenturi_Event, this));

	MEM_Ppatient_LP = new CircularBuffer(10);
	
	old_delta_ppatient = 0;
	
	sys_s.in_over_pressure_emergency = false;
	
	Alarms.Init(&MVM_HAL, &sys_s);

	MVM_SM.Init(&MVM_HAL, &CMC.core_config, &sys_s, 10);
	

}
void MVMCore::Tick()
{
	MVM_HAL.Tick();
	sys_s.pLoop = MVM_HAL.GetPressureValve(0);
	sys_s.pPatient = MVM_HAL.GetPressurePatient(0);
	sys_s.FlowIn = MVM_HAL.GetFlowInspire(0);
	sys_s.FlowVenturi = MVM_HAL.GetFlowVenturi(0);
	MVM_HAL.GetInputValvePID(&sys_s.pid_valvein_slow, &sys_s.pid_valvein_fast);
	MVM_SM.Tick();
}

bool MVMCore::DataAvailableOnUART0()
{
	return MVM_HAL.DataAvailableOnUART0();
}

String MVMCore::ReadUART0UntilEOL()
{
	return MVM_HAL.ReadUART0UntilEOL();
}

bool MVMCore::WriteUART0(String s)
{
	return MVM_HAL.WriteUART0(s);
}

bool MVMCore::SetParameter(String p, String v)
{
	return CMC.SetParameter(p, v);
}
String MVMCore::GetParameter(String p)
{
	return CMC.GetParameter(p);
}

void MVMCore::PLoop_Event()
{
	float v = MVM_HAL.GetPressureValve(0);

	//SAFETY
	if (v > PV1_SAFETY_LIMIT)
	{
		Alarms.Action_OverPressureSecurity();
	}
}
void MVMCore::PPatient_Event()
{
	float v = MVM_HAL.GetPressurePatient(0);
	sys_s.pPatient_low_passed = 0.90 * sys_s.pPatient_low_passed + v * 0.1;
	sys_s.pres_peak = sys_s.pPatient_low_passed > sys_s.pres_peak ?
							sys_s.pPatient_low_passed : sys_s.pres_peak;
	MEM_Ppatient_LP->PushData(sys_s.pPatient_low_passed);

	sys_s.PPatient_delta = v - MEM_Ppatient_LP->GetData(5);
	sys_s.PPatient_delta2 = sys_s.PPatient_delta - old_delta_ppatient;
	old_delta_ppatient = sys_s.PPatient_delta;

	//SAFETY
	if (v > PV1_SAFETY_LIMIT)
	{
		Alarms.Action_OverPressureSecurity();
	}

	//Serial.println("CALLBACK P: " + String(v));
}

void MVMCore::FlowIn_Event()
{
	float v = MVM_HAL.GetFlowInspire(0);
	sys_s.fluxpeak = sys_s.fluxpeak > v ? sys_s.fluxpeak : v;
	TidalVolumeExt.PushDataSens(v);
	sys_s.TidalVolume = TidalVolumeExt.liveVolume;
	sys_s.Flux = TidalVolumeExt.liveFlux;

	//Serial.println("CALLBACK F: " + String(v));
}

void MVMCore::FlowVenturi_Event()
{
	float vf;
	vf = MVM_HAL.GetFlowVenturi(0);
	TidalVolumeExt.PushDataVenturi(vf);
	sys_s.VenturiFlux = 0.8 * sys_s.VenturiFlux + 0.2 * vf;
	sys_s.dt_veturi_100ms= MVM_HAL.GetPVenturi(5);
	sys_s.TidalVolume = TidalVolumeExt.liveVolume;
	sys_s.Flux = TidalVolumeExt.liveFlux;
}


void MVMCore::NewCycle_Event()
{
	TidalVolumeExt.DoNewCycle();
	float dTact = (float)MVM_HAL.Get_dT_millis(last_respiratory_act);
	last_respiratory_act = MVM_HAL.GetMillis();

	if (dTact > 0)
	{
		sys_s.last_bpm = 60000.0 / dTact;
	}
	else
	{
		sys_s.last_bpm = 0;
	}
	sys_s.averaged_bpm = 0.8 * sys_s.averaged_bpm + 0.2 * sys_s.last_bpm;
}
void MVMCore::Exhale_Event()
{
	TidalVolumeExt.DoExhale();
	sys_s.currentTvIsnp = TidalVolumeExt.currentTvIsnp;
	sys_s.currentVM = sys_s.currentTvIsnp * sys_s.last_bpm;
}
void MVMCore::EndCycle_Event()
{
	TidalVolumeExt.DoEndCycle();
	sys_s.currentTvEsp = TidalVolumeExt.currentTvEsp;
}