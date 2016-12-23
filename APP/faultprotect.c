#include "faultprotect.h"

void SysLed_Twinkle(MOTOR_FAULT*pmotor_fault)
{
    pmotor_fault->Led_indicate.Timernumcount++;
    
    switch(pmotor_fault->Led_indicate.Systemstatus)
    {
        case SYSTATUS_STANDBY:      //待机
            {
                if( pmotor_fault->Led_indicate.Timernumcount<=STANDBY_ONTIME )
                    LED_ON;
                else if( pmotor_fault->Led_indicate.Timernumcount<STANDBY_OFFTIME )
                    LED_OFF;
                else
                    pmotor_fault->Led_indicate.Timernumcount = 0;
            }
            break;
        case SYSTATUS_NORMAL:       //正常
            {
                if( pmotor_fault->Led_indicate.Timernumcount<=NORMAL_ONTIME )
                    LED_ON;
                else if( pmotor_fault->Led_indicate.Timernumcount<NORMAL_OFFTIME )
                    LED_OFF;
                else
                    pmotor_fault->Led_indicate.Timernumcount = 0;
            }
            break;
        case SYSTATUS_OVERVOL:      //过压
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < OVER_VOL_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_LOWVOL:       //欠压
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < LOW_VOL_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_OVERPHASECURRENT:       //相电流过流
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < OVER_PHASECURRENT_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_LOSESTEP:       //失步
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < LOSE_STEP_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_SPEEDUNNORMAL:       //转速不对
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < SPEED_UNNORMAL_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_CURRENTDEVIATION:       //电流偏差过大
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < CURRENT_DEVIATION_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_IPM_FO:       //FO输入故障
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < IPM_FO_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_ADDEVIATION:       //AD异常
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < AD_DEVIATION_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        case SYSTATUS_OVERTEMPERATURE:       //过温
            {
                if( pmotor_fault->Led_indicate.Twinkletimes < OVER_TEMPERATURE_TWINKLETIMES )
                {
                    if( pmotor_fault->Led_indicate.Timernumcount<=FAULT_TWINKLETIME )
                        LED_ON;
                    else if( pmotor_fault->Led_indicate.Timernumcount<(2*FAULT_TWINKLETIME) )
                        LED_OFF;
                    else
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes++;
                    }
                }
                else
                {
                    LED_OFF;
                    if( pmotor_fault->Led_indicate.Timernumcount >= FAULT_OFFTIME )
                    {
                        pmotor_fault->Led_indicate.Timernumcount = 0;
                        pmotor_fault->Led_indicate.Twinkletimes = 0;
                    }
                }
            }
            break;
        default:
            {
                if( pmotor_fault->Led_indicate.Timernumcount<=STANDBY_ONTIME )
                    LED_ON;
                else if( pmotor_fault->Led_indicate.Timernumcount<STANDBY_OFFTIME )
                    LED_OFF;
                else
                {
                    pmotor_fault->Led_indicate.Timernumcount = 0;
                    pmotor_fault->Led_indicate.Twinkletimes = 0;
                }
            }
            break;
            
    }
}

void dcbus_voltage_protect(MOTOR_FAULT* pmotor_fault)    //过压、欠压检测
{
	static u32 dcbus_over_times = 0,dcbus_under_times = 0;

    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_OVERVOL \
        || pmotor_fault->fault_code == SYSTATUS_LOWVOL )
    {
        if(TargetDCbus >= DCbus_OVERVOLTAGE_ADVAL	 && ++dcbus_over_times >= DCbus_OVER_LOW_TIMES)
        {
            pmotor_fault->fault_code = SYSTATUS_OVERVOL;
            dcbus_over_times = DCbus_OVER_LOW_TIMES;
        }
        else if(TargetDCbus <= DCbus_UNDERVOLTAGE_ADVAL && ++dcbus_under_times >= DCbus_OVER_LOW_TIMES)
        {
            pmotor_fault->fault_code = SYSTATUS_LOWVOL;
            dcbus_under_times = DCbus_OVER_LOW_TIMES;
        }
        else if( (TargetDCbus < (DCbus_OVERVOLTAGE_ADVAL - DELTA_VALUE)) && (TargetDCbus > (DCbus_UNDERVOLTAGE_ADVAL+ DELTA_VALUE)) )
        {
            dcbus_over_times = 0;
                dcbus_under_times = 0;
            pmotor_fault->fault_code = SYSTATUS_NORMAL;
        }
    }
    
    View_Variable1 = TargetDCbus;
    View_Variable2 = DCbus_OVERVOLTAGE_ADVAL;
    View_Variable3 = DCbus_UNDERVOLTAGE_ADVAL;
    
}

void phase_current_max_check(MOTOR_FAULT* pmotor_fault)
{
    
	if( (pmotor_fault->Phase_current_detect.Phase_current_check_flag==0) \
        && (pmotor_fault->Phase_current_detect.Adsample_times++ < ELECTRIC_TIMER_ADSAMPLE_NUMBER) )
	{
		if( pmotor_fault->Phase_current_detect.Ia_max <= CorrADC0 )
			pmotor_fault->Phase_current_detect.Ia_max = CorrADC0;
		if( pmotor_fault->Phase_current_detect.Ib_max <= CorrADC1 )
			pmotor_fault->Phase_current_detect.Ib_max = CorrADC1; 

		if( pmotor_fault->Phase_current_detect.Ia_min >= CorrADC0 )
			pmotor_fault->Phase_current_detect.Ia_min = CorrADC0;
		if( pmotor_fault->Phase_current_detect.Ib_min >= CorrADC1 )
			pmotor_fault->Phase_current_detect.Ib_min = CorrADC1;
	}
	else
	{
		pmotor_fault->Phase_current_detect.Phase_current_check_flag = 1;    //检测完成
		pmotor_fault->Phase_current_detect.Adsample_times = 0;
	}

}

void phase_current_protect(MOTOR_FAULT* pmotor_fault)   //ok
{
	static u32 phase_protect_times = 0;

    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_OVERPHASECURRENT )
    {
        if( (pmotor_fault->Phase_current_detect.Ia_max > PHASE_OVERCURRENT || \
            pmotor_fault->Phase_current_detect.Ib_max > PHASE_OVERCURRENT) && (++phase_protect_times >= PHASE_OVERCURRENT_TIMES) )
        {
            pmotor_fault->fault_code = SYSTATUS_OVERPHASECURRENT;
            phase_protect_times = PHASE_OVERCURRENT_TIMES;
        }
        else if( (pmotor_fault->Phase_current_detect.Ia_max < (PHASE_OVERCURRENT - DELTA_VALUE)) || pmotor_fault->Phase_current_detect.Ib_max < (PHASE_OVERCURRENT - DELTA_VALUE) )
        {
            pmotor_fault->fault_code = SYSTATUS_NORMAL;
            phase_protect_times = 0;
        } 
    }

}

void motor_lose_step_detect(MOTOR_FAULT* pmotor_fault)   //ok,无电机检测
{
	static u32 phasecurrent_near_zero_number = 0;

    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_LOSESTEP )
    {
        if( ((pmotor_fault->Phase_current_detect.Ia_max < PHASECURRENT_NEAR_ZERO_ADVALUE)&&(pmotor_fault->Phase_current_detect.Ia_min> (0-PHASECURRENT_NEAR_ZERO_ADVALUE)))\
        || ((pmotor_fault->Phase_current_detect.Ib_max < PHASECURRENT_NEAR_ZERO_ADVALUE)&&(pmotor_fault->Phase_current_detect.Ib_min> (0-PHASECURRENT_NEAR_ZERO_ADVALUE))) )
            phasecurrent_near_zero_number++;
        else 
            phasecurrent_near_zero_number = 0;

        if( phasecurrent_near_zero_number >= PHASE_CURRENT_NEAR_ZERO_TIMES )
        {
            pmotor_fault->fault_code = SYSTATUS_LOSESTEP;
            phasecurrent_near_zero_number = PHASE_CURRENT_NEAR_ZERO_TIMES;
        }
        else
            pmotor_fault->fault_code = SYSTATUS_NORMAL;
    }
}

void lose_phase_detect(MOTOR_FAULT* pmotor_fault)    //转速不对
{
	static u32 motor_rotate_speed  = 0;
	static u32 last_motor_rotate_speed = 0;
	static u32 confirm_lose_phase_number = 0;
	
    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_SPEEDUNNORMAL )
    {
        motor_rotate_speed = fg_outfrequence * 60;
        if( (stas==2) && ((motor_rotate_speed > last_motor_rotate_speed + 60) || (motor_rotate_speed < last_motor_rotate_speed - 60)) )  //(motor_rotate_speed >> 3) ) \
            || (motor_rotate_speed < last_motor_rotate_speed - (motor_rotate_speed >> 3) ))
        {
            if( ++confirm_lose_phase_number >=UNNORMAL_SPEED_TIMES)
            {
                pmotor_fault->fault_code = SYSTATUS_SPEEDUNNORMAL;
                confirm_lose_phase_number = UNNORMAL_SPEED_TIMES;
            }
        }
        else
        {
            confirm_lose_phase_number = 0;
            pmotor_fault->fault_code = SYSTATUS_NORMAL;
        }	
        last_motor_rotate_speed = motor_rotate_speed;
    }
    
//    View_Variable1 = last_motor_rotate_speed;
//    View_Variable2 = motor_rotate_speed;
//    View_Variable3 = confirm_lose_phase_number;
}

void Phase_current_deviation(MOTOR_FAULT* pmotor_fault)
{
	static u32 confirm_phase_current_number=0;
	s16 Ia_amplitude,Ib_amplitude;
	s16 Ia_Ib_amplitude;

    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_CURRENTDEVIATION )
    {
        Ia_amplitude = gabs((pmotor_fault->Phase_current_detect.Ia_max - pmotor_fault->Phase_current_detect.Ia_min))>>1;
        Ib_amplitude = gabs((pmotor_fault->Phase_current_detect.Ib_max - pmotor_fault->Phase_current_detect.Ib_min))>>1;
        Ia_Ib_amplitude = gabs(Ia_amplitude - Ib_amplitude);
        
        if( stas && (((Ia_amplitude<Ib_amplitude)&&(Ia_Ib_amplitude*PHASE_CURRENT_DEVIATION_RATIO>Ia_amplitude)) \
        || ((Ia_amplitude>Ib_amplitude)&&(Ia_Ib_amplitude*PHASE_CURRENT_DEVIATION_RATIO>Ib_amplitude))) )
            confirm_phase_current_number++;
        else
            confirm_phase_current_number = 0; 

        if( confirm_phase_current_number >= PHASE_CURRENT_DEVIATION_TIMES )
        {
            pmotor_fault->fault_code = SYSTATUS_CURRENTDEVIATION;
            confirm_phase_current_number = PHASE_CURRENT_DEVIATION_TIMES;
        }
        else
            pmotor_fault->fault_code = SYSTATUS_NORMAL;
    }
    
//    View_Variable1 = Ia_amplitude;
//    View_Variable2 = Ib_amplitude;
//    View_Variable3 = Ia_Ib_amplitude*PHASE_CURRENT_DEVIATION_RATIO;
}

void IPM_FO_detect(MOTOR_FAULT* pmotor_fault)   //IPM_FO检测
{
	static u32 confirm_ipm_fo_number=0;
	u32 ipm_fo_status;

    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_IPM_FO )
    {
        ipm_fo_status = GET_IPM_FO_STATUS;  //读取IPM_FO引脚电平

        if( ipm_fo_status == 0 )
            confirm_ipm_fo_number++;
        else
            confirm_ipm_fo_number = 0;

        if( confirm_ipm_fo_number >=IPM_FO_LOW_TIMES )
        {
            pmotor_fault->fault_code = SYSTATUS_IPM_FO;
            confirm_ipm_fo_number = IPM_FO_LOW_TIMES;
        }
        else
            pmotor_fault->fault_code = SYSTATUS_NORMAL;  
    }
    
//    View_Variable1 = confirm_ipm_fo_number;   
}

void Phase_Current_Offset_Advalue_detect(MOTOR_FAULT* pmotor_fault) //电流检测点电平偏差过大
{
	s16 Ia_offset,Ib_offset;

    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_ADDEVIATION )
    {
        Ia_offset = MeasCurrParm.Offseta;
        Ib_offset = MeasCurrParm.Offsetb;

        if( ((Ia_offset < Ib_offset)&&(gabs(Ia_offset - Ib_offset)*PHASE_CURRENT_AD_DEVIATION_RATIO >= Ia_offset)) \
        || ((Ia_offset > Ib_offset)&&(gabs(Ia_offset - Ib_offset)*PHASE_CURRENT_AD_DEVIATION_RATIO >= Ib_offset)) )
            pmotor_fault->fault_code = SYSTATUS_ADDEVIATION;
        else
            pmotor_fault->fault_code = SYSTATUS_NORMAL; 
    }    
    
//    View_Variable1 = Ia_offset;
//    View_Variable2 = Ib_offset;
//    View_Variable3 = gabs(Ia_offset - Ib_offset)*PHASE_CURRENT_AD_DEVIATION_RATIO;       
}

void Over_temperature_detect(MOTOR_FAULT* pmotor_fault)     //过温检测
{
	static u32 confirm_over_temp_times;
    
    if( pmotor_fault->fault_code <= SYSTATUS_NORMAL || pmotor_fault->fault_code == SYSTATUS_OVERTEMPERATURE )
    {
        if( (pmotor_fault->IPM_module.Ad_Temperature_value >0) && (pmotor_fault->IPM_module.Ad_Temperature_value+200 < ADVALUE_OVER_TEMPERATUR) && (++confirm_over_temp_times>=OVER_TEMPERATURE_TIMES) )
        {
            pmotor_fault->fault_code = SYSTATUS_OVERTEMPERATURE;
            confirm_over_temp_times = OVER_TEMPERATURE_TIMES;
        }
        else
            pmotor_fault->fault_code = SYSTATUS_NORMAL;  
    }
}

void (*pfn[])(MOTOR_FAULT*) = 
{
	phase_current_protect,
	motor_lose_step_detect,
	Phase_current_deviation,
	lose_phase_detect,
	dcbus_voltage_protect,
	IPM_FO_detect,
	Phase_Current_Offset_Advalue_detect,
	Over_temperature_detect	
};
void motor_fault_init(MOTOR_FAULT *pmotor_fault)
{
	pmotor_fault->fault_code = 0;
	pmotor_fault->IPM_module.Ad_Temperature_value = ADVALUE_DEG25;
}


void motor_fault_detect(MOTOR_FAULT*pmotor_fault)
{
	static u32 fun_number = 0;
	static u32 fun_number1 = 3;
    if( (pmotor_fault->Phase_current_detect.Phase_current_check_flag == 1) && (fun_number<=2) )
    {
        pfn[fun_number++](pmotor_fault);
    }	
    else
    {
        if( fun_number >2 )
        {
            pfn[fun_number++](pmotor_fault);
            if(fun_number > 7)
            {
                fun_number = 0;	

                pmotor_fault->Phase_current_detect.Phase_current_check_flag = 0;                
                pmotor_fault->Phase_current_detect.Ia_max = 0;
                pmotor_fault->Phase_current_detect.Ia_min = 0;
                pmotor_fault->Phase_current_detect.Ib_max = 0;
                pmotor_fault->Phase_current_detect.Ib_min = 0;
            }	
        }
    } 	
}

