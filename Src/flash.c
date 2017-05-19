#include "stdint.h"
#include "flash.h"
#include "usart.h"
#include "stm32f0xx_hal.h"

extern RESPONSE rsp; 
extern COMMAND  cmd; 

FLASH_EraseInitTypeDef Calibration ;



void WriteFlashCurrentCalibration (void)
{
	uint16_t CalibrationCurrentData=cmd.clb_curfact;//2.55A
	if ((CalibrationCurrentData>>9)==85)//temp=1010101B;
	{
		uint16_t TempVoltageCalibrationData=ReadFlashVoltageCalibration();
		HAL_FLASH_Unlock();
		Calibration.TypeErase = FLASH_TYPEERASE_PAGES;//page earse only
		Calibration.PageAddress = PageCalibration;//the address of page is about to be erased
		Calibration.NbPages = 1;
		uint32_t PageEraseError = 0;
		HAL_FLASHEx_Erase(&Calibration,&PageEraseError);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, AddressCurrentCalibration, CalibrationCurrentData);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, AddressVoltageCalibration, TempVoltageCalibrationData);
		HAL_FLASH_Lock();
	}
}

uint16_t ReadFlashCurrentCalibration (void)
{
  uint16_t temp = *(uint16_t*)(AddressCurrentCalibration);
	return(temp);
}

void WriteFlashVoltageCalibration (void)
{
	uint16_t CalibrationVoltageData=cmd.clb_volfact;//22.5V
	if ((CalibrationVoltageData>>9)==85)//temp=1010101B;
	{
		uint16_t TempCurrentCalibrationData=ReadFlashCurrentCalibration();
		HAL_FLASH_Unlock();
		Calibration.TypeErase = FLASH_TYPEERASE_PAGES;//page earse only
		Calibration.PageAddress = PageCalibration;//the address of page is about to be erased
		Calibration.NbPages = 1;
		uint32_t PageEraseError = 0;
		HAL_FLASHEx_Erase(&Calibration,&PageEraseError);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, AddressCurrentCalibration, TempCurrentCalibrationData);
		HAL_FLASH_Program(TYPEPROGRAM_WORD, AddressVoltageCalibration, CalibrationVoltageData);	
		HAL_FLASH_Lock();
	}
}

uint16_t ReadFlashVoltageCalibration (void)
{
	uint16_t temp = (*(__IO uint16_t*)(AddressVoltageCalibration));
	return(temp);
}
