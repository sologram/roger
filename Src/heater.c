#include "heater.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

extern uint16_t task_delay[];
extern uint16_t pwm_pulse;
extern uint16_t target_power;

extern uint8_t flag_throttle;
extern uint8_t heatstep;
extern uint8_t flag_closepower;
extern uint8_t counter_syntony;

extern RESPONSE rsp;
extern COMMAND  cmd; 

extern uint16_t poweroff_delay;

static uint8_t counter_addpower_delay=0;

void task1_PowerControl(void)
{

	task_delay[1] = 1;
	heat_protect();	
	power_handle();
  heat_process();
	if (poweroff_delay>0)
		  poweroff_delay--;
}

void power_handle(void)
{
  static uint16_t  new_target_power,old_target_power,poweron_time,poweroff_time,counter_poweron,counter_poweroff;

		target_power = cmd.power;
		new_target_power = cmd.power;							
		if ((new_target_power > 0)&&(new_target_power < cmd.lmt_power))
			{
				if (old_target_power != new_target_power)
				{
					old_target_power = new_target_power;
					poweron_time=old_target_power*PowerDistributePeriod/cmd.lmt_power;
					if (poweron_time<200)
							poweron_time=200;
					if (poweron_time>(PowerDistributePeriod-200))
							poweron_time=PowerDistributePeriod-200;					
					poweroff_time=PowerDistributePeriod-poweron_time;
					counter_poweron=poweron_time;
					counter_poweroff=poweroff_time;
				}

				if (counter_poweron>0)//timer for targer power lower than 1000w
						counter_poweron--;	
				if ((counter_poweroff>0)&&(counter_poweron==0))
						counter_poweroff--;	
				if (counter_poweroff==0)
						{  
							counter_poweroff=poweroff_time;
							counter_poweron=poweron_time;
						}
				
				if (counter_poweron>0)
					target_power=cmd.lmt_power;
				else
					target_power=0;
				
			}		
			
    static uint8_t flag_poweroffcommand=0;		
		if ((target_power==0)&&(flag_poweroffcommand==0))
		{
			heatstep = 5;
			flag_poweroffcommand=1;
		}
			
		if((heatstep==0)&&(target_power>(cmd.lmt_power-1)))
		{
			heatstep = 1;	
			flag_poweroffcommand=0;
		}
}


void load_test(void)
{
	  if ((poweroff_delay==0)&&((rsp.error&0xff)==0))
				{
					pwm_pulse=StartHeatPwm;
					set_counter_pin_interrupt(1);		
					pwm_pulse_set(pwm_pulse);
					pwm_start();					
					uint8_t i;
					for (i=0;i<255;i++)
					{}					
					pwm_stop();					
          heatstep++;		
          counter_syntony=0;						
				}
}



void load_check(void)
{
	  set_counter_pin_interrupt(0);
	  rsp.loadtest=counter_syntony;
	   if ((rsp.loadtest<cmd.loadtest)&&(rsp.loadtest>2)) 
		   {
				  heatstep++;
				 	pwm_pulse=StartHeatPwm;
          pwm_pulse_set(pwm_pulse);
	        pwm_start();
				 	set_fan_speed(100);	
				  counter_addpower_delay=60;
			 }
		else
		{
      heat_stop_with_on_delay();
			ErrorReport(ERROR_NOPAN);
		}
}

void heat_start(void)
{	
		if (counter_addpower_delay>0)//waitting for voltage drop 
       counter_addpower_delay--;
	if (rsp.power<cmd.lmt_power)
			    {
						pwm_pulse_set(pwm_pulse++);
						if (pwm_pulse>600)
								{
									heat_stop_with_on_delay();
									ErrorReport(ERROR_NOPAN);
								}
						
					}
	else
	{
		if (counter_addpower_delay==0)
		      heatstep++;
	}
	
}

void heat_adjust(void)
{
	static uint8_t counter_add=0;
	
	if (counter_add>3)
		 counter_add=0;
	else
		 counter_add++;
	if ((rsp.power<target_power)&&(counter_add==0)&&(flag_throttle==0)&&(rsp.current<(cmd.lmt_current-40)))
			    {
						if ((target_power-rsp.power)>100)
							pwm_pulse_set(pwm_pulse++);
								
			    }
	if	(rsp.power>target_power)	 
			{ 	
				 if ((rsp.power-target_power>400))
             {
							 pwm_pulse=pwm_pulse-5;
							 pwm_pulse_set(pwm_pulse);
						 }				 
					
			}
 
	if (flag_throttle==1)
		 flag_throttle=0;

	
		rsp.pulse=pwm_pulse;			
}

void heat_stop_with_off_delay(void)//for case of normally power off such as power distribution and power off command
{
	if (flag_closepower)
	{
			pwm_stop();
			heatstep=0;	
			pwm_pulse=0;
      set_fan_speed(0);
	}
}

void heat_stop_with_on_delay(void)//for case of protection
{
	pwm_stop();
	heatstep=0;	
	pwm_pulse=0;
  poweroff_delay=nopan_delay;
	set_fan_speed(0);	
}

void heat_protect(void)
{
	

  if (heatstep==4)
	{
			if ((rsp.voltage<1800)||(rsp.voltage>cmd.lmt_voltage))	
			{
					heat_stop_with_on_delay();
					ErrorReport(ERROR_VOLTAGE);
			}
  }
	
	if (rsp.current>(cmd.lmt_current-20))//max current limitation
	{ 
		pwm_pulse_set(pwm_pulse--);
	}

	static uint16_t counter_movepandelay=0;
	if (heatstep==4)//check pan status
	 {
			if (rsp.power<(cmd.lmt_power-200))
					{
						counter_movepandelay++;
						if (counter_movepandelay>10000)
						{
							counter_movepandelay=0;
						  heat_stop_with_on_delay();
						  ErrorReport(ERROR_NOPAN);
						}
					}
      else
         	counter_movepandelay=0;			
			if (rsp.power==0)
			{
							counter_movepandelay=0;
						  heat_stop_with_on_delay();
						  ErrorReport(ERROR_NOPAN);
				
			}
		}

		
    static uint8_t flag_sink_overhot=0;	//heatsink overhot protect
		if (rsp.tempsink<cmd.lmt_tempsink)
			{
					flag_sink_overhot=1;		
					heat_stop_with_on_delay();
					ErrorReport(ERROR_SINKOVERHOT);
			}			
		if (flag_sink_overhot==1)
			{
				if (rsp.tempsink>(cmd.lmt_tempsink+300))
						flag_sink_overhot=0;
				else
				{
					ErrorReport(ERROR_SINKOVERHOT); 
					heat_stop_with_on_delay();
				}
			}		
		
}




void heat_process(void)
{	
				 switch (heatstep)
							{
									case 1://
									{
										load_test();
										break;
									}
									case 2://
									{
										load_check();
										break;
									}
									case 3://
									{  
									 heat_start();				
										break;
									}
									case 4://
									{
										heat_adjust();
										break;
									}
									case 5://
									{ 
										heat_stop_with_off_delay();
										break;
									}		
									default://
									{
										heatstep = 0;	
										break;
									}
					}
}

