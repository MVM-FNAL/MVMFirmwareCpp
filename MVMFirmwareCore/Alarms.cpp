// 
// 
// 

#include "Alarms.h"
#include "UtilsFn.h"

/**
 * \brief Initialize alarm class
 * 
 * This function mist be called before any other usages
 * of the class
 * 
 * \param hal       Pointer to the HAL class
 * \param sys_c     Pointer to the t_SystemStatus class
 */
void AlarmClass::Init(HAL* hal, t_SystemStatus* sys_c)
{
    //Make a private copy of HALL and SystemStatus pointers
	_sys_c = sys_c;
	_HAL = hal;

    //Class Variable initialization
    AlarmSound = false;
    led_on = false;
    isInAlarm = false;
    wdog_enable = false;

    //Create two circular buffer to store the dP on Valve and Patient
    CycleCyclePLoop = new CircularBuffer(5);
    CycleCyclePPatient = new CircularBuffer(5);
}


/**
 * \brief   Tick function to be called periodically
 * 
 * The function check for alarms and execute the task 
 * related to the alarm manage
 */
void AlarmClass::Tick()
{
    if (!_disable_alarms)
    {
        //Check alarms that are not triggered by events
        CheckStaticAlarms();

        //Filter alarm in function of snooze bits
        ALARM_FLAG_FILTERED = ALARM_FLAG & (~ALARM_FLAG_SNOOZE);

        //Reset snooze bits after 120s
        if (_HAL->Get_dT_millis(ALARM_FLAG_SNOOZE_millis) > 120000)
            ALARM_FLAG_SNOOZE = 0;

        if (ALARM_FLAG != 0) {
            isInAlarm = true;
        }
        else
        {
            isInAlarm = false;
        }

        if (ALARM_FLAG_FILTERED != 0) {
            AlarmSound = true;
        }
        else
        {
            AlarmSound = false;
        }

        //Buzzer
        Sound();

        //Other alarm actions
        AlarmActions();
    }
    //GUI alarm flags
    _sys_c->ALARM_FLAG = ALARM_FLAG_FILTERED;
    
}

/**
 * \brief   Leds and relays managment 
 * 
 *  Led RED blink 4 Hz
 * 
 *  Rele for high priority alarms 
 * 
 */
void AlarmClass::AlarmActions()
{
    if (isInAlarm)
    {
        if (_HAL->Get_dT_millis(blinker_led_time)>250)
        {
            blinker_led_time = _HAL->GetMillis();
            led_on = led_on ? false : true;
            _HAL->SetAlarmLed(led_on);
        }
        _HAL->SetAlarmRele(true);
    }
    else
    {
        led_on = false;
        _HAL->SetAlarmLed(false);
        _HAL->SetAlarmRele(false);
    }
}



/**
 * \brief   Execute the High Priority melody
 * 
 * The state machine execute step by step the ISO MELODY
 */
void AlarmClass::Sound()
{
	if (AlarmSound)
	{
        switch (alarm_state) {
        case 0:
            buzzer_time = _HAL->GetMillis();
            alarm_state = 1;
            _HAL->SetBuzzer(true); //#1
            break;

        case 1:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 2;
            }
            break;

        case 2:
            if (_HAL->Get_dT_millis(buzzer_time) > 100) {
                _HAL->SetBuzzer(true);  //#2
                buzzer_time = _HAL->GetMillis();
                alarm_state = 3;
            }
            break;

        case 3:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 4;
            }
            break;

        case 4:
            if (_HAL->Get_dT_millis(buzzer_time) > 100) {
                _HAL->SetBuzzer(true);  //#3
                buzzer_time = _HAL->GetMillis();
                alarm_state = 5;
            }
            break;

        case 5:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 6;
            }
            break;

        case 6:
            if (_HAL->Get_dT_millis(buzzer_time) > 300) {
                _HAL->SetBuzzer(true);  //#4
                buzzer_time = _HAL->GetMillis();
                alarm_state = 7;
            }
            break;

        case 7:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 8;
            }
            break;

        case 8:
            if (_HAL->Get_dT_millis(buzzer_time) > 100) {
                _HAL->SetBuzzer(true);  //#5
                buzzer_time = _HAL->GetMillis();
                alarm_state = 9;
            }
            break;

        case 9:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 10;
            }
            break;

        case 10:
            if (_HAL->Get_dT_millis(buzzer_time) > 800) {
                _HAL->SetBuzzer(true);  //#5
                buzzer_time = _HAL->GetMillis();
                alarm_state = 11;
            }
            break;

        case 11:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 12;
            }
            break;

        case 12:
            if (_HAL->Get_dT_millis(buzzer_time) > 100) {
                _HAL->SetBuzzer(true);  //#2
                buzzer_time = _HAL->GetMillis();
                alarm_state = 13;
            }
            break;

        case 13:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 14;
            }
            break;

        case 14:
            if (_HAL->Get_dT_millis(buzzer_time) > 100) {
                _HAL->SetBuzzer(true);  //#3
                buzzer_time = _HAL->GetMillis();
                alarm_state = 15;
            }
            break;

        case 15:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 16;
            }
            break;

        case 16:
            if (_HAL->Get_dT_millis(buzzer_time) > 300) {
                _HAL->SetBuzzer(true);  //#4
                buzzer_time = _HAL->GetMillis();
                alarm_state = 17;
            }
            break;

        case 17:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 18;
            }
            break;

        case 18:
            if (_HAL->Get_dT_millis(buzzer_time) > 100) {
                _HAL->SetBuzzer(true);  //#5
                buzzer_time = _HAL->GetMillis();
                alarm_state = 19;
            }
            break;

        case 19:
            if (_HAL->Get_dT_millis(buzzer_time) > 150) {
                _HAL->SetBuzzer(false);
                buzzer_time = _HAL->GetMillis();
                alarm_state = 20;
            }
            break;

        case 20:
            if (_HAL->Get_dT_millis(buzzer_time) > 10000) {
                alarm_state = 0;
            }
            break;
        }
	}
    else
    {
        _HAL->SetBuzzer(false);
        alarm_state = 0;
    }
}

/**
 * \brief   Over Pressure action: trigger alarm and open exhaust valve
 * 
 */
void AlarmClass::Action_OverPressureSecurity()
{
	_sys_c->in_over_pressure_emergency = true;
	_HAL->SetOutputValve(true);
}

/**
 * \brief check for alarms that are not triggered by events
 * 
 *      - Battery under 20%
 *      - Pressure @ Patient > 50mbar
 *      - Pressure @ Loop   > 65mbar
 *      - GUI watchdog
 */
void AlarmClass::CheckStaticAlarms()
{
    if ((_sys_c->currentBatteryCharge < 20) && (_sys_c->batteryPowered))
    {
        TriggerAlarm(BATTERY_LOW);
    }

    
    if (_sys_c->pPatient > 50)
    {
        TriggerAlarm(ALARM_PRESSURE_INSIDE_TOO_HIGH);
    }

    if (_sys_c->pLoop > 65)
    {
        TriggerAlarm(ALARM_PRESSURE_INSIDE_TOO_HIGH);
    }


    if (wdog_enable)
    {
        if (_HAL->Get_dT_millis(wdog_timer)>6000)
        {
            TriggerAlarm(ALARM_GUI_WDOG);
        }
    }
}

/**
 * \brief   Generate a test alarm
 * 
 * \param in_alarm  true to generate alarm
 */
void AlarmClass::SetAlarmTest(bool in_alarm)
{
    if (in_alarm)
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_ALARM_TEST);
    else
        ALARM_FLAG = 0x00000000;

    ALARM_FLAG_SNOOZE = 0;
}


/**
 * \brief   Event handler for alarm triggering
 * 
 * This function convert the alarm code from firmware enum t_ALARM
 * in a generic alarm code shared with GUI. See generic_definitions.h
 * for alarms coding.
 * 
 * \param Alarm     Alarm code from t_ALARM enum
 */
void AlarmClass::TriggerAlarm(t_ALARM Alarm)
{
    switch (Alarm) {
    case PRESSURE_DROP_INHALE:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " PRESSURE_DROP_INHALE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_LEAKAGE);
        break;

    case UNABLE_TO_READ_SENSOR_PRESSURE:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " UNABLE_TO_READ_SENSOR_PRESSURE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case UNABLE_TO_READ_SENSOR_FLUX:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " UNABLE_TO_READ_SENSOR_FLUX");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case UNABLE_TO_READ_SENSOR_VENTURI:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " UNABLE_TO_READ_SENSOR_VENTURI");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case ALARM_COMPLETE_OCCLUSION:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_COMPLETE_OCCLUSION");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_FULL_OCCLUSION);
        break;

    case ALARM_PARTIAL_OCCLUSION:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_PARTIAL_OCCLUSION");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_PARTIAL_OCCLUSION);
        break;

    case ALARM_PRESSURE_INSIDE_TOO_HIGH:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INSIDE_TOO_HIGH");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INSIDE_PRESSURE_HIGH);
        break;

    case ALARM_PRESSURE_INSIDE_TOO_LOW:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INSIDE_TOO_LOW");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INSIDE_PRESSURE_LOW);
        break;

    case ALARM_LEAKAGE:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_LEAKAGE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_LEAKAGE);
        break;

    case BATTERY_LOW:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " BATTERY_LOW");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_BATTERY_LOW);
        break;

    case ALARM_PRESSURE_INPUT_TOO_LOW:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INPUT_TOO_LOW");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INPUT_PRESSURE_LOW);
        break;

    case ALARM_PRESSURE_INPUT_TOO_HIGH:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INPUT_TOO_HIGH");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INPUT_PRESSURE_HIGH);
        break;

    case ALARM_GUI_ALARM:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_GUI_ALARM");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_ALARM_PI);
        break;

    case ALARM_GUI_WDOG:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_WDOG");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_WDOG_PI);
        break;

    case ALARM_OVER_UNDER_VOLTAGE:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_OVER_UNDER_VOLTAGE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;
    case ALARM_SUPERVISOR:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_SUPERVISOR");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case ALARM_OVERTEMPERATURE:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_OVERTEMPERATURE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case ALARM_APNEA:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_APNEA");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_APNEA);
        break;

	case ALARM_NO_VENTURI_CONNECTED:
		_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_APNEA");
		ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_VENTURI_NOT_CONNECTED);
		break;

	case ALARM_VENTURI_INVERTED:
		_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " ALARM_APNEA");
		ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_VENTURI_INVERTED);
		break;
        
    case UNPREDICTABLE_CODE_EXECUTION:
        _HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "ALARM @ " + String(millis()) + " UNPREDICTABLE_CODE_EXECUTION");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    default:

        break;
    }

    
}

/**
 * \brief   Trigger alarm from GUI
 * 
 * \param in_alarm     true to trigger alarm, false to snooze alarm
 */
void AlarmClass::SetAlarmGUI(bool in_alarm)
{
    if (in_alarm)
    {
        TriggerAlarm(ALARM_GUI_ALARM);
    }
    else   
    {
        ALARM_FLAG = ALARM_FLAG & (~GenerateFlag(__ERROR_ALARM_PI));
    }

}

/**
 * \brief   Reset and snooze all alarm
 */
void AlarmClass::ResetAlarm()
{
    ALARM_FLAG_SNOOZE = ALARM_FLAG ;
    ALARM_FLAG_SNOOZE_millis = _HAL->GetMillis();
    ALARM_FLAG = 0;
}

/**
 * \brief   Event triggered on start of breath cycle
 * 
 */
void AlarmClass::TransitionNewCycleEvent()
{
    P0Loop = _sys_c->pLoop;
    P0Patient = _sys_c->pPatient;
}

/**
 * \brief   Event triggered on start of exhale phase
 * 
 */
void AlarmClass::TransitionInhaleExhale_Event()
{
    float dPPatient, dPLoop;
    if (_sys_c->pLoop < 0.5 * _sys_c->current_pressure_setpoint)
    {
        TriggerAlarm(ALARM_PRESSURE_INSIDE_TOO_LOW);
    }

    if (_sys_c->pPatient < 0.8 * _sys_c->current_pressure_setpoint)
    {
        TriggerAlarm(ALARM_LEAKAGE);
    }

    dPPatient = fabs(_sys_c->pPatient - P0Patient);
    dPLoop = fabs(_sys_c->pLoop - P0Loop);

    CycleCyclePLoop->PushData(dPLoop);
    CycleCyclePPatient->PushData(dPPatient);

    if ((CycleCyclePLoop->GetData(1) > 2 * CycleCyclePPatient->GetData(1)) &&
        (CycleCyclePLoop->GetData(0) > 2 * CycleCyclePPatient->GetData(0)))
        TriggerAlarm(ALARM_PARTIAL_OCCLUSION);

    if ((CycleCyclePLoop->GetData(1) > 5) &&
            (CycleCyclePLoop->GetData(0) > 5) &&
            (CycleCyclePPatient->GetData(1) < 1.5) &&
            (CycleCyclePPatient->GetData(0) < 1.5))
        TriggerAlarm(ALARM_COMPLETE_OCCLUSION);

}

/**
 * \brief   Event triggered on end of breath cycle
 * 
 */
void AlarmClass::TransitionEndCycleEvent()
{

}

/**
 * \brief   Reset GUI watchdog
 * 
 */
void AlarmClass::ResetWatchDog()
{
    wdog_timer = _HAL->GetMillis();
}

/**
 * \brief   Enable GUI watchdog. Watchdog expire in 6s
 * 
 * \param enable    true: enable watchdog
 */
void AlarmClass::EnableWatchDog(bool enable)
{
    wdog_timer = _HAL->GetMillis();
    wdog_enable = enable;
}

/**
 * \brief   Service function to generate alarm MASKs
 * 
 * \param alarm_code    alarm code
 * \return              alarm mask
 */
uint32_t AlarmClass::GenerateFlag(int alarm_code)
{
    return (1 << alarm_code);
}



/**
 * \brief   Disable all alarms
 *
 * \param in_alarm  true to disable alarms
 */

void AlarmClass::DisableAllAlarms(bool disable)
{
    _disable_alarms = disable;
    ResetWatchDog();
    if (disable)
    {
        ALARM_FLAG_FILTERED = 0;
        ALARM_FLAG = 0;
        ALARM_FLAG_SNOOZE = 0;
        led_on = false;
        isInAlarm = false;
        AlarmSound = false;
        _HAL->SetAlarmLed(false);
        _HAL->SetAlarmRele(false);
        _HAL->SetBuzzer(false);

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
