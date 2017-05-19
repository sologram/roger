/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "heater.h"

/* USER CODE BEGIN 0 */
extern uint8_t flag_closepower;

extern uint8_t heatstep;


extern uint16_t task_delay[];
extern uint16_t ad_voltage;
extern uint16_t ad_current;
extern uint16_t ad_sink;
extern uint16_t ad_pan;
extern uint16_t poweroff_delay;

extern RESPONSE rsp; 
extern COMMAND  cmd; 
/* USER CODE END 0 */

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

static uint32_t ADCBuffer[16];

void task0_AdSample(void)
{
 task_delay[0]=1;

	static uint32_t ADCfilter[2];
	static uint8_t  pointer_adc=0;
	static uint8_t  flag_adc=0;
	static uint16_t filter_current[50],filter_voltage[50],filter_current1[50];
	uint8_t i;
	uint16_t realtime_ADvoltage,realtime_ADcurrent;
  uint32_t temp_current,temp_voltage;
	temp_current=0;
	temp_voltage=0;

	
	for(i=0;i<2;i++)
	     {ADCfilter[i]=ADCfilter[i]+ADCBuffer[i]+ADCBuffer[i+4]+ADCBuffer[i+8]+ADCBuffer[i+12];}
	realtime_ADvoltage=(ADCBuffer[2]+ADCBuffer[6]+ADCBuffer[10]+ADCBuffer[14])>>2;			 
	realtime_ADcurrent=(ADCBuffer[3]+ADCBuffer[7]+ADCBuffer[11]+ADCBuffer[15])>>2;

  /**judge the point to colse power by asbusolutly vlotage value ,this way can reduce action delay******/
			if (realtime_ADvoltage<400)
				 flag_closepower=1;
			else
				 flag_closepower=0;
			if (heatstep==5)
				   heat_stop_with_off_delay();			 
  /********/
			 
  if (pointer_adc<49)
		pointer_adc++;
	else
		pointer_adc=0;
	
	filter_current[pointer_adc]=realtime_ADcurrent;
	filter_current1[pointer_adc]=realtime_ADcurrent;
	filter_voltage[pointer_adc]=realtime_ADvoltage;	 
  /***judge the colse power point by tedency******/
	/*if (pointer_adc>1)
	   {
				if ((filter_voltage[pointer_adc]>filter_voltage[pointer_adc-1])&&(filter_voltage[pointer_adc-2]>filter_voltage[pointer_adc-1]))
						flag_closepower=1;
				else
						flag_closepower=0;
			}
  if (pointer_adc==1)
	   {
				if ((filter_voltage[1]>filter_voltage[0])&&(filter_voltage[49]>filter_voltage[0]))
						flag_closepower=1;
				else
						flag_closepower=0;
			}		
  if (pointer_adc==0)
	   {
				if ((filter_voltage[0]>filter_voltage[49])&&(filter_voltage[48]>filter_voltage[49]))
						flag_closepower=1;
				else
						flag_closepower=0;
			}*/
			
		 /***for colsing power output without noise quickly, at least saving 1ms***/

			/******/
			
			/*if (flag_closepower)
				HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);*/
		 
		 
	for(i=0;i<50;i++)
		 {
			 temp_current=temp_current+filter_current1[i];
			 temp_voltage=temp_voltage+filter_voltage[i];
		 }		
  ad_current=temp_current/50;
	ad_voltage=temp_voltage/50;	 
	temp_current=filter_current[0];//usless instruction,just for avoide warning
		 
	if (flag_adc==50)
			{
				flag_adc=0;
								
				ad_pan=ADCfilter[0]/200;
				ad_sink =ADCfilter[1]/200;
				ADCfilter[0]=0;
				ADCfilter[1]=0;
			}
 flag_adc++;			
			
	if (ad_current<250)
		rsp.current=0;
	else
    rsp.current=(ad_current*51-3533)/100;//unit 0.01A
	if (cmd.clb_curfact&0x100)
			rsp.current=rsp.current-(cmd.clb_curfact&0xff);
	else
			rsp.current=rsp.current+(cmd.clb_curfact&0xff);

	
	rsp.voltage=(1188*ad_voltage+30000)/1000;//unit 0.1V
	if (cmd.clb_volfact&0x100)
		rsp.voltage=rsp.voltage-(cmd.clb_volfact&0xff);
  else
		rsp.voltage=rsp.voltage+(cmd.clb_volfact&0xff);  

	
	rsp.power=rsp.current*rsp.voltage/100;//unit 0.1W
	
	rsp.tempsink=ad_sink;
	rsp.temppan=ad_pan;



/*rsp.temppan=ad_pan;
rsp.tempcoil=ad_coil;			
rsp.tempsink=ad_sink;
rsp.voltage=ad_voltage;
rsp.current=ad_current;*/



}





/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */	
	hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc,ADCBuffer,16);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
  
    /**ADC GPIO Configuration    
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    PA5     ------> ADC_IN5
    PA7     ------> ADC_IN7 
    */
    GPIO_InitStruct.Pin = AD_pan_Pin|AD_sink_Pin|AD_voltage_Pin|AD_current_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    /* Peripheral DMA init*/
  
    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    PA5     ------> ADC_IN5
    PA7     ------> ADC_IN7 
    */
    HAL_GPIO_DeInit(GPIOA, AD_pan_Pin|AD_sink_Pin|AD_voltage_Pin|AD_current_Pin);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
}   

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
