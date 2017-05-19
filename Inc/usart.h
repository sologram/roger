/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
HAL_StatusTypeDef HAL_UART_DMA_RX_Stop(UART_HandleTypeDef *huart);
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
void InstructionDecode(void);
void InstructionExecute(void);
void Parameters_Init(void);
void ErrorReport(uint16_t errorcode);
void ErrorReset(void);
void PostToTxdata(void);
/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_USART1_UART_Init(void);

#define __HAL_UART_RX_ENABLE(__HANDLE__)               ((__HANDLE__)->Instance->CR1 |=  USART_CR1_RE)
#define __HAL_UART_RX_DISABLE(__HANDLE__)              ((__HANDLE__)->Instance->CR1 &=  ~USART_CR1_RE)
#define __HAL_UART_TX_ENABLE(__HANDLE__)               ((__HANDLE__)->Instance->CR1 |=  USART_CR1_TE)
#define __HAL_UART_TX_DISABLE(__HANDLE__)              ((__HANDLE__)->Instance->CR1 &=  ~USART_CR1_TE)

  #define     MAX_CommBuffer  0x7f
	#define     MAX_RxKey       0x4d
/* USER CODE BEGIN Prototypes */
	#define   ERROR_SINKOVERHOT        0x0001
	#define   ERROR_VOLTAGE            0x0002
	#define   ERROR_COMFAILURE         0x0004	
	#define   ERROR_SURGE              0x0008
	
	#define   ERROR_NOPAN              0x0010

	#define   ERROR_OVERCURRENT        0x0100
	#define   ERROR_OVERVOLTAGE        0x0200
	#define   ERROR_INITFAILURE        0x0400

	#define   CMD_POWER_H        0x22
	#define   CMD_POWER_L        0x23
	#define   CMD_PERIPHERAL_H   0x24
	#define   CMD_PERIPHERAL_L   0x25
	#define   CMD_LOADTEST       0x26
	#define   CMD_GETDEFAULTS    0x27
	#define   CMD_LOG            0x28
	#define   CMD_ID             0x29
	
	#define   CLB_CURBASE_H        0x32
	#define   CLB_CURBASE_L  		   0x33
	#define   CLB_CURFACT_H        0x34
	#define   CLB_CURFACT_L        0x35
	#define   CLB_VOLBASE_H        0x36
	#define   CLB_VOLBASE_L        0x37
	#define   CLB_VOLFACT_H        0x38
	#define   CLB_VOLFACT_L        0x39
	
	#define   LMT_RSPLENGTH        0x42
	#define   LMT_RESERVED1        0x43
	#define   LMT_CURRENT_H        0x44
	#define   LMT_CURRENT_L        0x45
	#define   LMT_POWER_H          0x46
	#define   LMT_POWER_L          0x47
	#define   LMT_PULSE_H          0x48
	#define   LMT_PULSE_L          0x49
	#define   LMT_TEMPSINK_H       0x4a
	#define   LMT_TEMPSINK_L       0x4b
	#define   LMT_VOLTAGE_H        0x4c
	#define   LMT_VOLTAGE_L        0x4d
	#define   LMT_FREQUENCY_H      0x4e
	#define   LMT_FREQUENCY_L      0x4f
	
	#define   RSP_ERROR_H        0x62
	#define   RSP_ERROR_L        0x63
	#define   RSP_VOLTAGE_H      0x64
	#define   RSP_VOLTAGE_L      0x65
	#define   RSP_CURRENT_H      0x66
	#define   RSP_CURRENT_L      0x67
	#define   RSP_PULSE_H        0x68
	#define   RSP_PULSE_L        0x69
	#define   RSP_FREQUENCY_H    0x6a//
	#define   RSP_FREQUENCY_L    0x6b//
	#define   RSP_TEMPPAN_H      0x6c
  #define   RSP_TEMPPAN_L      0x6d	
	#define   RSP_TEMPCOIL_H     0x6e//
	#define   RSP_TEMPCOIL_L     0x6f//	
	#define   RSP_TEMPSINK_H     0x70
	#define   RSP_TEMPSINK_L     0x71
	
  #define   RSP_LOADTEST       0x72	
  #define   RSP_TOPOLOGY       0x73
		
	#define   RSP_VERSION_H      0x74
	#define   RSP_VERSION_L      0x75
	#define   RSP_CLOCK_H        0x76
	#define   RSP_CLOCK_L        0x77
	#define   RSP_LOG_H          0x78
	#define   RSP_LOG_L          0x79

	
typedef struct{
								int16_t  power;			  // 0x22//target power
								int16_t  peripheral;		// 0x24//fan and led control
								int8_t  loadtest;			// 0x26//reference load strength
								int8_t  log;			    // 0x28//
								int8_t  id;			      // 0x29//
								int16_t reserved1;		// 0x2a//
								int16_t reserved2;		// 0x2c//
								int16_t reserved3;		// 0x2e//
								int16_t reserved4;		// 0x30//
								int16_t clb_curbase;	// 0x32//
								int16_t clb_curfact;	// 0x34//
								int16_t clb_volbase;	// 0x36//
								int16_t clb_volfact;	// 0x38//
								int16_t reserved5;		// 0x3a//
								int16_t reserved6;		// 0x3c//
								int16_t reserved7;		// 0x3e//
								int16_t reserved8;		// 0x40//
								int16_t lmt_rsplength;// 0x42//returning data length
								int16_t lmt_current;	// 0x44//max current
								int16_t lmt_power;		// 0x46//min continuously heat power
								int16_t lmt_pulse;		// 0x48//max pwm pulse
								int16_t lmt_tempsink;	// 0x4a//max heatsink temperatue
								int16_t lmt_voltage;	// 0x4c//max voltage
             }COMMAND;

typedef struct{
								int16_t error;			// 0x62//error code 
								int16_t voltage;		// 0x64//actual voltage
								int16_t current;		// 0x66//actual current
								int16_t pulse;      // 0x68//actual pwm pulse
								int16_t frequency;	// 0x6a//actual syntony frequency
								int16_t tempsink;		// 0x6c//ad value of sink temperatue
								int16_t tempcoil;		// 0x6e//ad value of coil temperatue
	              int16_t temppan;   // 0x70//ad value of pan temperatue
								int8_t  loadtest;		// 0x76//acture load strength	
								int8_t  topology;	  // 0x77//1 half bridge or 0 single switch
								int16_t version;		// 0x78//
								int16_t reserved1;	// 0x80//								
								int16_t clock;			// 0x82//
	              int16_t log;        // 0x84//
                int16_t power;      // 0x85//actual heat power
              }RESPONSE;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
