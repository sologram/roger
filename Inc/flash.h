#ifndef __flash_H
#define __flash_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void WriteFlashCurrentCalibration (void);
uint16_t ReadFlashCurrentCalibration (void);

void WriteFlashVoltageCalibration (void);
uint16_t ReadFlashVoltageCalibration (void);
/* USER CODE END Prototypes */
#define PageCalibration             0x08003c00
#define AddressCurrentCalibration   0x08003c00
#define AddressVoltageCalibration   0x08003c02
#ifdef __cplusplus
}
#endif
#endif
