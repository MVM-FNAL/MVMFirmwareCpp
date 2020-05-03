// 
// 
// 

#include "MVM_StateMachine.h"


void MVM_StateMachine::Init(HAL* _MVM_HAL, AlarmClass *_MVM_Alarms, t_config* _core_config, t_SystemStatus* _sys_c, int32_t _dT)
{
    MVM_HAL = _MVM_HAL;
    MVM_Alarms = _MVM_Alarms;
	core_config = _core_config;
	sys_c = _sys_c;
	cycle_SMTick = MVM_HAL->GetMillis();
    timer1 = 0;
    mvm_sm = FR_OPEN_INVALVE;
    
    ddT = _dT;
}
void MVM_StateMachine::Tick()
{
	if (MVM_HAL->Get_dT_millis(cycle_SMTick)> ddT)
	{
        dT = MVM_HAL->Get_dT_millis(cycle_SMTick);
        cycle_SMTick = MVM_HAL->GetMillis();
		SMExecute();
	}
}

void MVM_StateMachine::SMExecute()
{

    static int delta_error_counter = 0;


    
    timer1+=dT;
    

    switch (mvm_sm) {
    case FR_OPEN_INVALVE:
        dbg_state_machine = 0;
        if (core_config->run) {
            //RUN RESPIRATORY
            if (core_config->BreathMode == M_BREATH_FORCED) {

                if (callback_NewCycle)
                    callback_NewCycle();
                //AUTOMATIC
                //PRES_SENS_CT[2].ZERO += dt_veturi_100ms;

                //ResetStats();
                
                last_start = MVM_HAL->GetMillis();

				if (sys_c->backup_apnea == false)
				{
					cycle_pressure_target = core_config->target_pressure_auto;
					core_config->inhale_ms = 60000.0 / core_config->respiratory_rate * (1 - core_config->respiratory_ratio);
					core_config->exhale_ms = 60000.0 / core_config->respiratory_rate * (core_config->respiratory_ratio);
				}
				else
				{
					cycle_pressure_target = core_config->apnea_ptarget;
					core_config->inhale_ms = 60000.0 / core_config->apnea_rate * (1 - core_config->apnea_ratio);
					core_config->exhale_ms = 60000.0 / core_config->apnea_rate * (core_config->apnea_ratio);
				}
              
                //Compensate pressure flat top error changing set point
                if (core_config->enable_pressure_compensation)
                {
                    if ((cycle_pressure_target - sys_c->currentP_Peak) > 1)
                        sys_c->pressure_compensation_coeff++;
                    else
                        if ((cycle_pressure_target - sys_c->currentP_Peak) < 1)
                           sys_c->pressure_compensation_coeff--;

                    sys_c->pressure_compensation_coeff > 15 ? 15 : sys_c->pressure_compensation_coeff;
                    sys_c->pressure_compensation_coeff < -15 ? -15 : sys_c->pressure_compensation_coeff;
                   
                }
                else
                {
                    sys_c->pressure_compensation_coeff = 0;
                }
            

                //TRY TO AUTOCOMPENSATE
                /*if (fabs(sys_c->pres_peak - core_config->target_pressure_auto) > 10)
                {
                    delta_error_counter++;
                }

              
                if (delta_error_counter>3)
                {
                    if (core_config->I == core_config->I_rhard)
                    {
                        core_config->I = core_config->I_rlow;
                        core_config->P = core_config->P_rlow;
                    }
                    else
                    {
                        core_config->I = core_config->I_rhard;
                        core_config->P = core_config->P_rhard;
                    }

                    delta_error_counter = 0;

                    MVM_HAL->ConfigureInputValvePID(core_config->P,
                        core_config->I,
                        core_config->D,
                        core_config->P2,
                        core_config->I2,
                        core_config->D2,
                        core_config->pid_limit
                    );
                }*/



                sys_c->__stat_param.t90a = 0;
                MVM_HAL->SetOutputValve(false);
                MVM_HAL->SetInputValve(cycle_pressure_target + sys_c->pressure_compensation_coeff);

                timer1 = dT;
                mvm_sm = FR_WAIT_INHALE_TIME;
                MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: Start automatic cycle");
            }
            else {
                //ASSISTED
                if (core_config->BreathMode == M_BREATH_ASSISTED) {
                    if (callback_EndCycle)
                        callback_EndCycle();
                   
                    MVM_HAL->SetOutputValve(true);
 
                    sys_c->dbg_trigger = 0;
                    //Trigger for assist breathing
                    

                    if (core_config->backup_enable)
                    {
                        float dt = MVM_HAL->Get_dT_millis( last_start);
                        if (dt / 1000.0 > core_config->backup_min_rate)
                        {
                            //backup_trigger = true;
                            core_config->BreathMode = M_BREATH_FORCED;
                            sys_c->backup_apnea = true;
                            sys_c->pressure_compensation_coeff = 0;
                            MVM_Alarms->TriggerAlarm(ALARM_APNEA);
                        }
                    }
                    if (((-1.0 * sys_c->PPatient_delta2) > core_config->assist_pressure_delta_trigger) && (sys_c->PPatient_delta < 0)) {
                        sys_c->dbg_trigger = 1;

                        if (callback_NewCycle)
                            callback_NewCycle();
                        //PRES_SENS_CT[2].ZERO += dt_veturi_100ms;

                        //StatEndCycle();
                        //ResetStats();

                        last_start = MVM_HAL->GetMillis();
                        cycle_pressure_target = core_config->target_pressure_assist;

						//Compensate pressure flat top error changing set point
						if (core_config->enable_pressure_compensation)
						{
							if ((cycle_pressure_target - sys_c->currentP_Peak) > 1)
								sys_c->pressure_compensation_coeff++;
							else
								if ((cycle_pressure_target - sys_c->currentP_Peak) < 1)
									sys_c->pressure_compensation_coeff--;

							sys_c->pressure_compensation_coeff > 15 ? 15 : sys_c->pressure_compensation_coeff;
							sys_c->pressure_compensation_coeff < -15 ? -15 : sys_c->pressure_compensation_coeff;

						}
						else
						{
							sys_c->pressure_compensation_coeff = 0;
						}

                        MVM_HAL->SetInputValve(cycle_pressure_target);
                        MVM_HAL->SetOutputValve(false);
                        timer1 = dT;

                        mvm_sm = AST_WAIT_MIN_INHALE_TIME;
                        MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: Start assisted cycle");
                    }
                }
                else {
                    //WE SHOULD NEVER GO HERE
                    MVM_Alarms->TriggerAlarm(UNPREDICTABLE_CODE_EXECUTION);
                }
            }
        }
        else {
            //WE ARE NOT RUNNING
            last_start = MVM_HAL->GetMillis();
            MVM_HAL->SetInputValve(0);
            MVM_HAL->SetOutputValve(true);
            sys_c->pressure_compensation_coeff = 0;
            sys_c->backup_apnea = false;
        }
        break;

    case FR_WAIT_INHALE_TIME:
        dbg_state_machine = 1;

        if (timer1 >= core_config->inhale_ms ) {

            if ((core_config->pause_inhale == false) && (core_config->pause_lg == false)) {
                timer1 = 0;

                if (callback_Exhale)
                    callback_Exhale();

                //StatPhaseExpire();

                MVM_HAL->SetInputValve(core_config->leak_compensation);
                MVM_HAL->SetOutputValve(true);

                mvm_sm = FR_WAIT_EXHALE_TIME;
                MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: FR_WAIT_EXHALE_TIME");
            }
            else {
                if (core_config->pause_lg == false)
                    MVM_HAL->SetInputValve(0);
                else {
                    MVM_HAL->SetInputValve(core_config->pause_lg_p);
                    core_config->pause_lg_timer -= dT;
                    if (core_config->pause_lg_timer <= 0)
                        core_config->pause_lg = false;
                }
            }
        }
        break;

    case FR_WAIT_EXHALE_TIME:
        dbg_state_machine = 2;

        if (timer1 >= core_config->exhale_ms ) {
            if (core_config->pause_exhale == false) {
                if (callback_EndCycle)
                    callback_EndCycle();
                //StatEndCycle();
                MVM_HAL->SetOutputValve(false);
                mvm_sm = FR_OPEN_INVALVE;
                MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: FR_OPEN_INVALVE");
            }
            else {
                MVM_HAL->SetOutputValve(false);
            }
        }
        else
        {
            if (timer1 >= 700 ) {

                if (core_config->pcv_trigger_enable)
                {
                    if (((-1.0 * sys_c->PPatient_delta2) > core_config->assist_pressure_delta_trigger) && (sys_c->PPatient_delta < 0)) {
                        //StatEndCycle();
                        if (callback_EndCycle)
                            callback_EndCycle();
                        MVM_HAL->SetOutputValve(false);
                        mvm_sm = FR_OPEN_INVALVE;
                    }
                }
            }
        }
        break;

    case AST_WAIT_MIN_INHALE_TIME:
        dbg_state_machine = 3;
        if (timer1 > 300) {
            if ((sys_c->pLoop >= core_config->target_pressure * 0.5) 
                    || (timer1 > 1000) ){
                mvm_sm = AST_WAIT_FLUX_DROP;
                MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: AST_WAIT_FLUX_DROP");
                timer1 = 0;
            }
        }
        break;

    case AST_WAIT_FLUX_DROP:
        dbg_state_machine = 4;

        if ((sys_c->FlowIn  <= (core_config->flux_close * sys_c->fluxpeak) / 100.0)
            || (sys_c->in_over_pressure_emergency == true)
            || (timer1 > 6000 ) ){
            last_isp_time = timer1;
            mvm_sm = AST_WAIT_FLUX_DROP_b;
            MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: AST_WAIT_FLUX_DROP_b");
        }
        break;

    case AST_WAIT_FLUX_DROP_b:
        dbg_state_machine = 5;
        if ((core_config->pause_inhale == false) && (core_config->pause_lg == false)) {
            timer1 = dT;
            
            //StatPhaseExpire();
            
            if (callback_Exhale)
                callback_Exhale();

            MVM_HAL->SetInputValve(core_config->leak_compensation);
            MVM_HAL->SetOutputValve(true);
            mvm_sm = AST_DEADTIME;
            MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: AST_DEADTIME");
        }
        else {
            if (core_config->pause_lg == false)
                MVM_HAL->SetOutputValve(false);
            else {
                MVM_HAL->SetInputValve(core_config->pause_lg_p);
                core_config->pause_lg_timer -= dT;
                if (core_config->pause_lg_timer <= 0)
                    core_config->pause_lg = false;
            }
        }
        break;

    case AST_DEADTIME:
        dbg_state_machine = 6;
        float dead_time_s;
        dead_time_s = 0.5 * last_isp_time;
        dead_time_s = dead_time_s > 400 ? dead_time_s : 400;
        dead_time_s = dead_time_s > 2000 ? 2000 : dead_time_s;
        if (timer1 >= dead_time_s) {
            if (core_config->pause_exhale == false) {
                mvm_sm = FR_OPEN_INVALVE;
                MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: FR_OPEN_INVALVE");
            }
            else {
                mvm_sm = AST_PAUSE_EXHALE;
                MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: AST_PAUSE_EXHALE");
            }
        }
        break;

    case AST_PAUSE_EXHALE:
        dbg_state_machine = 7;
        if (core_config->pause_exhale == false) {
            mvm_sm = FR_OPEN_INVALVE;
        }
        else {
            if (((-1.0 * sys_c->PPatient_delta2) > core_config->assist_pressure_delta_trigger) && (sys_c->PPatient_delta < 0)) {
                //PRES_SENS_CT[2].ZERO += dt_veturi_100ms;
                MVM_HAL->SetInputValve(0);
                MVM_HAL->SetOutputValve(false);
                mvm_sm = AST_PAUSE_EXHALEb;
                MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: AST_PAUSE_EXHALEb");
            }
        }
        break;

    case AST_PAUSE_EXHALEb:
        if (core_config->pause_exhale == false) {
            mvm_sm = FR_OPEN_INVALVE;
            MVM_HAL->dbg.DbgPrint(DBG_CODE, DBG_INFO, "SM: FR_OPEN_INVALVE");
        }
        break;
    default:
        dbg_state_machine = 1000;
        MVM_Alarms->TriggerAlarm(UNPREDICTABLE_CODE_EXECUTION);
        mvm_sm = FR_OPEN_INVALVE;
        break;
    }

    if (core_config->pause_timeout > 0)
    {
        core_config->pause_timeout-=dT;

    }
    else
    {
        core_config->pause_inhale = false;
        core_config->pause_exhale = false;
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
