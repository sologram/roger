/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
extern uint16_t task_delay[];

static uint8_t fan_status;
static uint8_t powerbus_status;

static uint16_t fan_delay;
static uint16_t powerbus_delay;


void fan_control(void);
void powerbus_control(void);
void SetFan(uint8_t status,uint16_t delay);
void SetPowerBus(uint8_t status,uint16_t delay);

void task4_PeripheralControl(void)
{
	 task_delay[4]=50;
	 
	 //fan_control();
	 powerbus_control();


}

void SetFan(uint8_t status,uint16_t delay)//open fan when status>0 and delay>0
{

	fan_status=status;
  fan_delay=delay;

}

void SetPowerBus(uint8_t status,uint16_t delay)//open fan when status>0 and delay>0
{

	powerbus_status=status;
  powerbus_delay=delay;

}



void fan_control(void)//if fan_delay=0xffff,never colse fan
{
	if ((fan_delay>0)&&(fan_delay<0xffff))
		  fan_delay--;
	if ((fan_status>0)&&(fan_delay>0))
		  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_SET);
	else 
		  HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_RESET);
}

void powerbus_control(void)//if fan_delay=0xffff,never colse fan
{
	if ((powerbus_delay>0)&&(powerbus_delay<0xffff))
		  powerbus_delay--;
	if ((powerbus_status>0)&&(powerbus_delay>0))
		  HAL_GPIO_WritePin(PowerBus_GPIO_Port, PowerBus_Pin, GPIO_PIN_SET);
	else 
		  HAL_GPIO_WritePin(PowerBus_GPIO_Port, PowerBus_Pin, GPIO_PIN_RESET);
}


/*void SetLed(uint8_t status,uint8_t delay)//open fan when status>0 and delay>0
{
	led_status=status;
	if (delay==0)
		  led_status=0;
	
	if (led_delay!=delay)
	{
			led_on_delay=delay;
			led_off_delay=delay;
			led_delay=delay;
	}
}
void led_control(void)
{
	if (led_status>0)
			{
			  if (led_on_delay==0)
				{
					HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
					
					if (led_off_delay>0)
							led_off_delay--;
					else
					{
						led_on_delay=led_delay;
						led_off_delay=led_delay;
					}
				}
				
				if (led_on_delay>0)
				{
					if (led_on_delay<0xff)
					    led_on_delay--;
					HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
				}
									
			}
	else
		  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
}*/
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Fan_Pin|PowerBus_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PFPin PFPin */
  GPIO_InitStruct.Pin = Fan_Pin|PowerBus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Surge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Surge_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = Counter_Pin|Throttle_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void set_counter_pin_interrupt(uint8_t status)
{
	if (status>0)
			  EXTI->IMR = EXTI->IMR | 0x2000;//enable exti_line13
	else
			  EXTI->IMR = EXTI->IMR & 0xffffdfff;//disable exti_line13
}
	

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
