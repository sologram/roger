/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Fan_Pin GPIO_PIN_1
#define Fan_GPIO_Port GPIOF
#define PowerBus_Pin GPIO_PIN_0
#define PowerBus_GPIO_Port GPIOF
#define AD_pan_Pin GPIO_PIN_0
#define AD_pan_GPIO_Port GPIOA
#define AD_sink_Pin GPIO_PIN_1
#define AD_sink_GPIO_Port GPIOA
#define Fan_pwm_Pin GPIO_PIN_4
#define Fan_pwm_GPIO_Port GPIOA
#define AD_voltage_Pin GPIO_PIN_5
#define AD_voltage_GPIO_Port GPIOA
#define Break_Pin GPIO_PIN_6
#define Break_GPIO_Port GPIOA
#define AD_current_Pin GPIO_PIN_7
#define AD_current_GPIO_Port GPIOA
#define PWM_H_Pin GPIO_PIN_1
#define PWM_H_GPIO_Port GPIOB
#define Surge_Pin GPIO_PIN_9
#define Surge_GPIO_Port GPIOA
#define PWM_L_Pin GPIO_PIN_10
#define PWM_L_GPIO_Port GPIOA
#define Counter_Pin GPIO_PIN_13
#define Counter_GPIO_Port GPIOA
#define Throttle_Pin GPIO_PIN_14
#define Throttle_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
