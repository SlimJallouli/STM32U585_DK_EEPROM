/**
  ******************************************************************************
  * @file    stm32u5xx_hal_pcd_ex.h
  * @author  MCD Application Team
  * @brief   Header file of PCD HAL Extension module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32U5xx_HAL_PCD_EX_H
#define STM32U5xx_HAL_PCD_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal_def.h"

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/** @addtogroup STM32U5xx_HAL_Driver
  * @{
  */

/** @addtogroup PCDEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup PCDEx_Exported_Functions PCDEx Exported Functions
  * @{
  */
/** @addtogroup PCDEx_Exported_Functions_Group1 Peripheral Control functions
  * @{
  */

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size);
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size);
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


HAL_StatusTypeDef HAL_PCDEx_ActivateLPM(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCDEx_DeActivateLPM(PCD_HandleTypeDef *hpcd);

#if defined (STM32U575xx) || defined (STM32U585xx) || defined (STM32U595xx) || defined (STM32U5A5xx) || defined (STM32U599xx) || defined (STM32U5A9xx)
HAL_StatusTypeDef HAL_PCDEx_ActivateBCD(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCDEx_DeActivateBCD(PCD_HandleTypeDef *hpcd);
void HAL_PCDEx_BCD_VBUSDetect(PCD_HandleTypeDef *hpcd);
#endif /* defined (STM32U575xx) || defined (STM32U585xx) || defined (STM32U595xx) || defined (STM32U5A5xx) || defined (STM32U599xx) || defined (STM32U5A9xx) */
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

#ifdef __cplusplus
}
#endif


#endif /* STM32U5xx_HAL_PCD_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
