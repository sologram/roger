#include "stdint.h"
#include "gpio.h"

extern void task0_AdSample(void);
extern void task1_PowerControl(void);
extern void task2_UartReceive(void);
extern void task3_UartSend(void);
extern void task4_PeripheralControl(void);
extern void task5_ErrorHandle(void);
/*task scheduler based on time distribution  

*/
//////////////////////////////////////////////////////////////////////
#define	MAX_TASK		6

uint16_t task_delay[MAX_TASK];

static void (*ptask[])()={task0_AdSample,task1_PowerControl,task2_UartReceive,task3_UartSend,task4_PeripheralControl,task5_ErrorHandle}; //fetch the PC of each task founction

void task_countdown(void)
{
	uint8_t i;
  for(i=0;i<MAX_TASK;i++)
	  {
			if((task_delay[i]>0)&&(task_delay[i]<0xffff))// Set task_delay[i] as 0xffff to suspend founction and never execute
				task_delay[i]--;
		}
}	
	
void task_scheduler(void) 
{
	uint8_t i;
	for(i=0;i<MAX_TASK;i++)
	     task_delay[i]=0;
	
  static uint8_t schedule_step;
	while (1) 
		{
       for(schedule_step=0;schedule_step<MAX_TASK;schedule_step++)
			     {
						 if (task_delay[schedule_step]==0)						 
						    {
									ptask[schedule_step]();						 
						      break;
								}
					 }
       
		}
	
}
//////////////////////////////////////////////////////////////////////

