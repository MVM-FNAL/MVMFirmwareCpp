// 
// 
// 

#include "ConfigManager.h"
#include "FwVersion.h"
#include "generic_definitions.h"
#include "UtilsFn.h"
#include "MVMCore.h"

/**
 * \brief   Initialize Configuration Manager class and setup the default MVM configuration
 * 
 * This function set the MVM configuration to default parameters. The class require access
 * to following class in MVM system:
 *      MVMcore         to execute function like flush_pipe, etc...
 *      SystemStatus    to get info like pressure, flow, alarms
 *      AlarmClass      to trigger GUI alarms ad watchdog
 * 
 * \param _core         Pointer to MVMcore, void to avoid forward class declaration
 * \param _sys_s        Pointer to System status stuct
 * \param _Alarms       Pointer to Alarm class
 */
void ConfigManagerClass::Init(void* _core, t_SystemStatus* _sys_s, AlarmClass *_Alarms)
{
    sys_s = _sys_s;
    core = _core;
    Alarms = _Alarms;
    core_config.run = false;
    core_config.inhale_ms = 750;
    core_config.exhale_ms = 1250;
    core_config.BreathMode = M_BREATH_FORCED; //M_BREATH_ASSISTED;//;
    core_config.flux_close = 5;
    core_config.assist_pressure_delta_trigger = 15;
    core_config.target_pressure = 20;
    core_config.target_pressure_auto = core_config.target_pressure;
    core_config.target_pressure_assist = core_config.target_pressure;

    core_config.respiratory_rate = 15;
    core_config.respiratory_ratio = 0.66;
    core_config.inhale_ms = 60000.0 / core_config.respiratory_rate * (1 - core_config.respiratory_ratio);
    core_config.exhale_ms = 60000.0 / core_config.respiratory_rate * (core_config.respiratory_ratio);

    core_config.apnea_ptarget = 15;
    core_config.apnea_rate = 20;
    core_config.apnea_ratio = 0.66;


    core_config.P = 150;
    core_config.I = 8;
    core_config.D = 0;

    core_config.I_rhard = 3;
    core_config.I_rlow= 15;
    core_config.P_rhard = 30;
    core_config.P_rlow = 70;

    core_config.P2 = 2.0;
    core_config.I2 = 0.2;
    core_config.D2 = 0;

    core_config.pid_limit = 0.0;

    core_config.pause_lg = false;
    core_config.pause_lg_timer = 0;

    core_config.__ADDTimeStamp = true;
    core_config.__CONSOLE_MODE = false;

}

/**
 * \brief   Function called by Set Parameters interface in order to program registers
 * 
 * ie.  set ptarget 30
 * 
 * \param p     Name of the parameters to set (ie. ptarget)
 * \param v     Value of the parameter (ie. 30)
 * \return      true if success, false if invalid parameter name or invalid value
 */
bool ConfigManagerClass::SetParameter(String p, String v)
{
    if (callback_BeforeConfigurationSet)
    {
        callback_BeforeConfigurationSet();
    }
    bool bres = false;
    String strPatam = p;

    // Start ventilation
    if (strPatam == "run") {
        int numberValue = v.toInt();

        if (numberValue < 1)
            core_config.run = false;
        else
            core_config.run = true;

        bres = true;
    }

    //Set ventilation mode
    //  0: PCV
    //  1: PSV
    if (strPatam == "mode") {
        int numberValue = v.toInt();
        if (numberValue == 0) {
            //Forced Mode
            core_config.BreathMode = M_BREATH_FORCED;
        }
        else {
            //Assisted Mode
            core_config.BreathMode = M_BREATH_ASSISTED;
        }
        bres = true;
    }

    //PCV: Set rate in bpm
    if (strPatam == "rate") {
        float numberValue = v.toFloat();
        core_config.respiratory_rate = numberValue;
        bres = true;
    }

    //PCV: Set ratio I:E
    if (strPatam == "ratio") {
        float numberValue = v.toFloat();
        core_config.respiratory_ratio = numberValue;
        bres = true;
    }

	//PCV: Set pressure target
	if (strPatam == "ptarget") {
		float numberValue = v.toFloat();
		core_config.target_pressure_auto = numberValue;
		bres = true;
	}

	//PCV: Enable trigger in PCV
	if (strPatam == "pcv_trigger_enable") {
		int numberValue = v.toInt();
		core_config.pcv_trigger_enable = numberValue ? true : false;
		Serial.println("valore=OK");
	}

	//PCV: trigger threshold in mbar/s^2
	if (strPatam == "pcv_trigger") {
		float numberValue = v.toFloat();
		core_config.pcv_trigger = numberValue;
		Serial.println("valore=OK");
	}


    //PSV: Set trigger in mbar/s^2
    if (strPatam == "assist_ptrigger") {
        float numberValue = v.toFloat();
        core_config.assist_pressure_delta_trigger = numberValue;
        bres = true;
    }

    //PSV: Set flow threshold
    if (strPatam == "assist_flow_min") {
        float numberValue = v.toFloat();
        core_config.flux_close = numberValue;
        bres = true;
    }
  
    //PSV: Set pressure support
    if (strPatam == "pressure_support") {
        float numberValue = v.toFloat();
        core_config.target_pressure_assist = numberValue;
        bres = true;
    }

	//PSV: Enable backup to PCV
	if (strPatam == "backup_enable") {
		int numberValue = v.toInt();
		core_config.backup_enable = numberValue ? true : false;
		bres = true;
	}

	//PSV: time between two breath to trigger backup
	if (strPatam == "backup_min_time") {
		float numberValue = v.toFloat();
		numberValue = numberValue < 1 ? 1 : numberValue;
		core_config.backup_min_rate = numberValue;
		bres = true;
	}

    //Configure PID: P component
    if (strPatam == "pid_p") {
        float numberValue = v.toFloat();
        core_config.P = numberValue;
        bres = true;
    }

    //Configure PID: I component
    if (strPatam == "pid_i") {
        float numberValue = v.toFloat();
        core_config.I = numberValue;
        bres = true;
    }

    //Configure PID: D component
    if (strPatam == "pid_d") {
        float numberValue = v.toFloat();
        core_config.D = numberValue;
        bres = true;
    }

    //Configure PID: P2 (SLOW) component
    if (strPatam == "pid_p2") {
        float numberValue = v.toFloat();
        core_config.P2 = numberValue;
        bres = true;
    }

    //Configure PID: I2 (SLOW) component
    if (strPatam == "pid_i2") {
        float numberValue = v.toFloat();
        core_config.I2 = numberValue;
        bres = true;
    }

    //Configure PID: D2 (SLOW) component
    if (strPatam == "pid_d2") {
        float numberValue = v.toFloat();
        core_config.D2 = numberValue;
        bres = true;
    }

    //Pause inhale. Must be called every 250ms
    if (strPatam == "pause_inhale") {
        int numberValue = v.toInt();
        core_config.pause_inhale = numberValue;
        core_config.pause_timeout = 500 ;
        bres = true;
    }

	//Pause exhale. Must be called every 250ms
	if (strPatam == "pause_exhale") {
		int numberValue = v.toInt();
		core_config.pause_exhale = numberValue;
		core_config.pause_timeout = 500;
		bres = true;
	}

    //Start Lecrutament procedure
    if (strPatam == "pause_lg") {
        int numberValue = v.toInt();
        core_config.pause_lg = numberValue ? true : false;
        bres = true;
    }

    //Lecrutament procedure time
    if (strPatam == "pause_lg_time") {
        int numberValue = v.toFloat();
        core_config.pause_lg_timer = numberValue * 1000.0;
        bres = true;
    }

    //Lecrutament procedure pressure
    if (strPatam == "pause_lg_p") {
        int numberValue = v.toFloat();
        core_config.pause_lg_p = numberValue;
        bres = true;
    }

    //Force PID SLOW to limit negative range
    if (strPatam == "pid_limit") {
        float numberValue = v.toFloat();
        core_config.pid_limit = numberValue;
        bres = true;
    }

    //Snooze alarms
    if (strPatam == "alarm_snooze") {
        int numberValue = v.toInt();
        //Alarm 29 is the GUI alarm
        if (numberValue == 29)
        {
            Alarms->SetAlarmGUI(false);
        }
        else
            Alarms->ResetAlarm();
        bres = true;
    }

    //Trigger Alarm from GUI
    if (strPatam == "alarm") {
        int numberValue = v.toInt();
        Alarms->SetAlarmGUI(numberValue != 0 ?true:false);
        bres = true;
    }

	//Trigger Alarm from testing
	if (strPatam == "alarm_test") {
		int numberValue = v.toInt();
		Alarms->SetAlarmTest(numberValue != 0 ? true : false);
		bres = true;
	}

    //Reset GUI watchdog
    if (strPatam == "watchdog_reset") {
        int numberValue = v.toInt();
        Alarms->ResetWatchDog();
        sys_s->ALARM_FLAG = sys_s->ALARM_FLAG & (~GenerateFlag(__ERROR_WDOG_PI));
        bres = true;
    }

	//Enable GUI watchdog
	if (strPatam == "wdenable") {
		int numberValue = v.toInt();
		core_config.__WDENABLE = numberValue != 0 ? true : false;
		Alarms->ResetWatchDog();
		Alarms->EnableWatchDog(core_config.__WDENABLE);
		bres = true;
	}

    //Enable console print debug
    if (strPatam == "console") {
        int numberValue = v.toInt();
        core_config.__CONSOLE_MODE = numberValue != 0 ? true : false;
        bres = true;
    }

    //Enable timestamp in console
    if (strPatam == "timestamp") {
        int numberValue = v.toInt();
        core_config.__ADDTimeStamp = numberValue != 0 ? true : false;
        bres = true;
    }

    if (strPatam == "stats_clear") {
        //ResetStatsBegin();
        bres = true;
    }

    //Start flush pipe procedure 
    //This command regulate the valve to the value specified in
    //the argument
    if (strPatam == "flush_pipe") {
        float numberValue = v.toFloat();
        bool enable = numberValue < 1 ? false:true;
        ((MVMCore*)core)->FlushPipes(enable, numberValue);
        bres = true;
    }

    //Enable leak compensation using an electronic peep
    if (strPatam == "leak_compensation") {
        float numberValue = v.toFloat();
        core_config.leak_compensation = numberValue;
        bres = true;
    }
    
    //Use cycle to cycle plateau pressure value compensation
    if (strPatam == "epc") {
        float numberValue = v.toFloat();
        bool enable = numberValue < 1 ? false : true;
        core_config.enable_pressure_compensation = enable;
        bres = true;
    }

    //PSV: When PSV backup to PCV this parameter specify the PCV rate
	if (strPatam == "apnea_rate") {
		float numberValue = v.toFloat();
		core_config.apnea_rate = numberValue;
		bres = true;
	}

    //PSV: When PSV backup to PCV this parameter specify the PCV ratio
	if (strPatam == "apnea_ratio") {
		float numberValue = v.toFloat();
		core_config.apnea_ratio = numberValue;
		bres = true;
	}

    //PSV: When PSV backup to PCV this parameter specify the PCV pressure
	if (strPatam == "apnea_ptarget") {
		float numberValue = v.toFloat();
		core_config.apnea_ptarget = numberValue;
		bres = true;
	}

    //When using CUSTOM Venturi, set fitting coefficient 0
	if (strPatam == "venturi_coefficient_0") {
		float numberValue = v.toFloat();
        ((MVMCore*)core)->VenturiSetCoefficient(0, numberValue);
		bres = true;
	}

    //When using CUSTOM Venturi, set fitting coefficient 1
	if (strPatam == "venturi_coefficient_1") {
		float numberValue = v.toFloat();
		((MVMCore*)core)->VenturiSetCoefficient(1, numberValue);
		bres = true;
	}

    //When using CUSTOM Venturi, set fitting coefficient 2
	if (strPatam == "venturi_coefficient_2") {
		float numberValue = v.toFloat();
		((MVMCore*)core)->VenturiSetCoefficient(2, numberValue);
		bres = true;
	}

    //When using CUSTOM Venturi, set fitting coefficient 3
	if (strPatam == "venturi_coefficient_3") {
		float numberValue = v.toFloat();
		((MVMCore*)core)->VenturiSetCoefficient(3, numberValue);
		bres = true;
	}

    //When using CUSTOM Venturi, set fitting coefficient 4
	if (strPatam == "venturi_coefficient_4") {
		float numberValue = v.toFloat();
		((MVMCore*)core)->VenturiSetCoefficient(4, numberValue);
		bres = true;
	}

    //Call this event at end of configuration to update/reset internal processes
	if (callback_AfterConfigurationSet)
	{
		callback_AfterConfigurationSet();
	}


    return bres;
}


/**
 * \brief   Function called by Get Parameters interface in order to read registers
 *
 * ie.  get ptarget
 *      get flow
 * 
 * \param p     Name of the parameter
 * \return      String to be sent to the GUI. String format is valure=value
 */
String ConfigManagerClass::GetParameter(String p)
{
    if (callback_BeforeConfigurationGet)
        callback_BeforeConfigurationGet();
    String strPatam = p;

    
    //Get pressure on valve
    if (strPatam == "pressure") {
        return "valore=" + String(sys_s->pLoop);
   
    }

    //Get pressure at patient
    if (strPatam == "ppressure") {
        return "valore=" + String(sys_s->pPatient);

    }

    //Get flow from Sensirion
    if (strPatam == "flow") {
        return "valore=" + String(sys_s->FlowIn);

    }

    //Get FiO2 value
    if (strPatam == "o2") {
        return "valore=" + String(sys_s->last_O2);
        
    }

    //Get rate in bpm (measured)
    if (strPatam == "bpm") {
        return "valore=" + String(sys_s->last_bpm);
    }

    //Check if system in PSV commutated in PCV for backup
    if (strPatam == "backup") {
        return "valore=" + String(sys_s->backup_apnea);
    }

    //Get tidal volume
    if (strPatam == "tidal") {
        return "valore=" + String(sys_s->TidalVolume);
    }

    //Get measure peep value
    if (strPatam == "peep") {
        return "valore=" + String(sys_s->last_peep);
    }

    //Get gas temperature
    if (strPatam == "temperature") {
        return "valore=" + String(sys_s->GasTemperature);
    }

    //Get power mode: 1 battery powered, 0 power wall
    if (strPatam == "power_mode") {
        return "valore=" + String(sys_s->batteryPowered ? 1 : 0);
    }

    //Get battery status
    if (strPatam == "battery") {
        return "valore=" + String(sys_s->currentBatteryCharge);
    }

    //Get Firmware version
    if (strPatam == "version") {
        return "valore=" + String(_FIRMWARE_VERSION_);
    }

    //Get alarm flags
    if (strPatam == "alarm") {
        return "valore=" + String(sys_s->ALARM_FLAG);
    }

    //Get warning flags
    if (strPatam == "warning") {
        return "valore=" + String(sys_s->WARNING_FLAG);
    }

    //Get run status
    if (strPatam == "run") {
        return "valore=" + String(core_config.run ? 1 : 0);
    }

    //Get run mode
    //0: PCV
    //1: PSV
    if (strPatam == "mode") {
        return "valore=" + String(core_config.BreathMode == M_BREATH_ASSISTED ? 1 : 0);
    }

    //Get rate in bpm
    if (strPatam == "rate") {
        return "valore=" + String(core_config.respiratory_rate);
    }

    //Get ratio I:E
    if (strPatam == "ratio") {
        return "valore=" + String(core_config.respiratory_ratio);
    }

    //Get PSV pressure trigger 
    if (strPatam == "assist_ptrigger") {
        return "valore=" + String(core_config.assist_pressure_delta_trigger);
    }

    //Get PSV flow threshold
    if (strPatam == "assist_flow_min") {
        return "valore=" + String(core_config.flux_close);
    }

    //Get PCV pressure target
    if (strPatam == "ptarget") {
        return  "valore=" + String(core_config.target_pressure_auto);
    }

    //Get PSV pressure target
    if (strPatam == "pressure_support") {
        return  "valore=" + String(core_config.target_pressure_assist);
    }

    //Get PSV backup enable/disable
    if (strPatam == "backup_enable") {
        return  "valore=" + String(core_config.backup_enable ? 1 : 0);

    }

    //Get PSV backup apnea time
    if (strPatam == "backup_min_rate") {
        return  "valore=" + String(core_config.backup_min_rate);
    }

    //Get Lectrutament pause state
    if (strPatam == "pause_lg") {
        return  "valore=" + String(core_config.pause_lg);
    }

    //Get Lectrutament pause expire time
    if (strPatam == "pause_lg_time") {
        return  "valore=" + String(core_config.pause_lg_timer / 1000.0);
    }

    //Get Lectrutament pause pressure
    if (strPatam == "pause_lg_p") {
        return  "valore=" + String(core_config.pause_lg_p);
    }

    //Get leak compensation target
    if (strPatam == "leak_compensation") {
        return  "valore=" + String(core_config.leak_compensation);
    }

    //Get apnea rate when PSV backup to PCV
	if (strPatam == "apnea_rate") {
		return  "valore=" + String(core_config.apnea_rate);
	}

    //Get apnea ratio when PSV backup to PCV
	if (strPatam == "apnea_ratio") {
		return  "valore=" + String(core_config.apnea_ratio);
	}

    //Get apnea pressure when PSV backup to PCV
	if (strPatam == "apnea_ptarget") {
		return  "valore=" + String(core_config.apnea_ptarget);
	}

    //Get used by GUI to download all parameters together
    /*
        0       patient pressure
        1       flux
        2       o2
        3       respiratory rate
        4       real time volume
        5       peep
        6       gas temperature
        7       power status
        8       battery status
        9       plateau pressure
        10      inspired volume
        11      expired volume
        12      Minute Volume
    */
    if (strPatam == "all") {
        return "valore=" + String(sys_s->pPatient) 
            + "," + String(sys_s->Flux)
            + "," + String(sys_s->last_O2)
            + "," + String(sys_s->last_bpm)
            + "," + String(sys_s->TidalVolume) 
            + "," + String(sys_s->last_peep)
            + "," + String(sys_s->GasTemperature) 
            + "," + String(sys_s->batteryPowered ? 1 : 0) 
            + "," + String(sys_s->currentBatteryCharge)
            + "," + String(sys_s->currentP_Peak)
            + "," + String(sys_s->currentTvIsnp*1000.0)
            + "," + String(sys_s->currentTvEsp * 1000.0)
            + "," + String(sys_s->currentVM);
    }

    //Calibrate all pressure sensor to 0 mbar
    if (strPatam == "calib") {
        float zeros[4];
        int count=4;
        ((MVMCore*)core)->ZeroSensors(zeros,&count);
        String outval ="valore=";
        for (int i = 0; i < count; i++)
        {
            if (i != count - 1)
                outval += String(zeros[i]) + ",";
            else
                outval += String(zeros[i]);
        }
        return outval;
    }

    //calibrare o2 sensore to AIR
    if (strPatam == "calib_o2") {
        ((MVMCore*)core)->CalibrateOxygenSensor();
        return "valore=OK";
    }

    if (strPatam == "calibv") {
        /*if (fabs(tidal_volume_c.ExpVolumeVenturi) > 0)
            tidal_volume_c.AutoZero = fabs(tidal_volume_c.InspVolumeVenturi) / fabs(tidal_volume_c.ExpVolumeVenturi);

        Serial.println("valore=" + String(tidal_volume_c.InspVolumeVenturi) + "," + String(tidal_volume_c.ExpVolumeVenturi) + "," + String(tidal_volume_c.AutoZero));
    
*/    }

    if (strPatam == "stats") {
        /*if (__stat_param.mean_cnt > 0) {
            float overshoot_avg = __stat_param.overshoot_avg / __stat_param.mean_cnt;
            float overshoot_length_avg = __stat_param.overshoot_length_avg / __stat_param.mean_cnt;
            float final_error_avg = __stat_param.final_error_avg / __stat_param.mean_cnt;
            float t1050_avg = __stat_param.t1050_avg / __stat_param.mean_cnt;
            float t1090_avg = __stat_param.t1090_avg / __stat_param.mean_cnt;
            float tpeak_avg = __stat_param.tpeak_avg / __stat_param.mean_cnt;
            float t9010_avg = __stat_param.t9010_avg / __stat_param.mean_cnt;
            float t9050_avg = __stat_param.t9050_avg / __stat_param.mean_cnt;
            float peep_avg = __stat_param.peep_avg / __stat_param.mean_cnt;
            float t10_avg = __stat_param.t10_avg / __stat_param.mean_cnt;
            float time_to_peak_avg = __stat_param.time_to_peak_avg / __stat_param.mean_cnt;
            float flux_peak_avg = __stat_param.flux_peak_avg / __stat_param.mean_cnt;
            float flux_t1090_avg = __stat_param.flux_t1090_avg / __stat_param.mean_cnt;
            float flux_t9010_avg = __stat_param.flux_t9010_avg / __stat_param.mean_cnt;

            Serial.println("valore=overshoot_avg:" + String(overshoot_avg)
                + ",overshoot_length_avg:" + String(overshoot_length_avg)
                + ",final_error:" + String(final_error_avg)
                + ",t1050_avg:" + String(t1050_avg)
                + ",t1090_avg:" + String(t1090_avg)
                + ",tpeak_avg:" + String(tpeak_avg)
                + ",t9010_avg:" + String(t9010_avg)
                + ",t9050_avg:" + String(t9050_avg)
                + ",peep_avg:" + String(peep_avg)
                + ",t10_avg:" + String(t10_avg)
                + ",time_to_peak_avg:" + String(time_to_peak_avg)
                + ",flux_peak_avg:" + String(flux_peak_avg)
                + ",flux_t1090_avg:" + String(flux_t1090_avg)
                + ",flux_t9010_avg:" + String(flux_t9010_avg));
        }
        else {
            Serial.println("valore=no_data");
        }*/
    }

    //Get flow Sensirion and dP on venturi
    if (strPatam == "get_fp") {
        return  "valore=" + String(sys_s->FlowIn) + "," + String(sys_s->VenturiP);
    }

    //Start a flow scan for venturi calibration
    if (strPatam == "venturi_scan") {
        ((MVMCore*)core)->DOVenturiMeterScan();
        return  "valore=OK";
    }

    //Start flow scan for valve calibration
    if (strPatam == "valve_scan") {
        ((MVMCore*)core)->DOValveScan();
        return  "valore=OK";
    }

    //Start leakage test
	if (strPatam == "leakage_test") {
		((MVMCore*)core)->LEAKAGETest();
        return  "valore=OK";
	}

    

    return "valore=ERROR:Invalid Command Argument";
}

uint32_t ConfigManagerClass::GenerateFlag(int alarm_code)
{
    return (1 << alarm_code);
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
