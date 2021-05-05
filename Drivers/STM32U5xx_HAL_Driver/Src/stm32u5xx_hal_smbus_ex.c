/**
  ******************************************************************************
  * @file    stm32u5xx_hal_smbus_ex.c
  * @author  MCD Application Team
  * @brief   SMBUS Extended HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of SMBUS Extended peripheral:
  *           + Extended features functions
  *
  @verbatim
  ==============================================================================
               ##### SMBUS peripheral Extended features  #####
  ==============================================================================

  [..] Comparing to other previous devices, the SMBUS interface for STM32U5xx
       devices contains the following additional features

       (+) Disable or enable wakeup from Stop mode(s)
       (+) Disable or enable Fast Mode Plus

                     ##### How to use this driver #####
  ==============================================================================
    (#) Configure the enable or disable of SMBUS Wake Up Mode using the functions :
          (++) HAL_SMBUSEx_EnableWakeUp()
          (++) HAL_SMBUSEx_DisableWakeUp()
    (#) Configure the enable or disable of fast mode plus driving capability using the functions :
          (++) HAL_SMBUSEx_ConfigFastModePlus()
    (#) Set or get or clear the autonomous mode configuration using these functions :
          (++) HAL_SMBUSEx_SetConfigAutonomousMode()
          (++) HAL_SMBUSEx_GetConfigAutonomousMode()
          (++) HAL_SMBUSEx_ClearConfigAutonomousMode()
  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/** @addtogroup STM32U5xx_HAL_Driver
  * @{
  */

/** @defgroup SMBUSEx SMBUSEx
  * @brief SMBUS Extended HAL module driver
  * @{
  */

#ifdef HAL_SMBUS_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup SMBUSEx_Exported_Functions SMBUS Extended Exported Functions
  * @{
  */

/** @defgroup SMBUSEx_Exported_Functions_Group1 Extended features functions
  * @brief    Extended features functions
 *
@verbatim
 ===============================================================================
                      ##### Extended features functions #####
 ===============================================================================
    [..] This section provides functions allowing to:
      (+) Configure Wake Up Feature

      (+) Configure Fast Mode Plus

@endverbatim
  * @{
  */

/**
  * @brief  Enable SMBUS wakeup from Stop mode(s).
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUSEx_EnableWakeUp(SMBUS_HandleTypeDef *hsmbus)
{
  /* Check the parameters */
  assert_param(IS_I2C_WAKEUP_FROMSTOP_INSTANCE(hsmbus->Instance));

  if (hsmbus->State == HAL_SMBUS_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_BUSY;

    /* Disable the selected SMBUS peripheral */
    __HAL_SMBUS_DISABLE(hsmbus);

    /* Enable wakeup from stop mode */
    hsmbus->Instance->CR1 |= I2C_CR1_WUPEN;

    __HAL_SMBUS_ENABLE(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Disable SMBUS wakeup from Stop mode(s).
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUSEx_DisableWakeUp(SMBUS_HandleTypeDef *hsmbus)
{
  /* Check the parameters */
  assert_param(IS_I2C_WAKEUP_FROMSTOP_INSTANCE(hsmbus->Instance));

  if (hsmbus->State == HAL_SMBUS_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_BUSY;

    /* Disable the selected SMBUS peripheral */
    __HAL_SMBUS_DISABLE(hsmbus);

    /* Disable wakeup from stop mode */
    hsmbus->Instance->CR1 &= ~(I2C_CR1_WUPEN);

    __HAL_SMBUS_ENABLE(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Configure SMBUS Fast Mode Plus.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @param  FastModePlus New state of the Fast Mode Plus.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUSEx_ConfigFastModePlus(SMBUS_HandleTypeDef *hsmbus, uint32_t FastModePlus)
{
  /* Check the parameters */
  assert_param(IS_SMBUS_ALL_INSTANCE(hsmbus->Instance));
  assert_param(IS_SMBUS_FAST_MODE_PLUS(FastModePlus));

  if (hsmbus->State == HAL_SMBUS_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_BUSY;

    /* Disable the selected SMBUS peripheral */
    __HAL_SMBUS_DISABLE(hsmbus);

    if (FastModePlus == SMBUS_FASTMODEPLUS_ENABLE)
    {
      /* Set SMBUSx FMP bit */
      hsmbus->Instance->CR1 |= (I2C_CR1_FMP);
    }
    else
    {
      /* Reset SMBUSx FMP bit */
      hsmbus->Instance->CR1 &= ~(I2C_CR1_FMP);
    }

    __HAL_SMBUS_ENABLE(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Set Autonomous Mode configuration
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @param  sConfig Pointer to a SMBUS_AutonomousModeConfTypeDef structure that contains
  *                the configuration information of the autonomous mode for the specified SMBUSx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUSEx_SetConfigAutonomousMode(SMBUS_HandleTypeDef *hsmbus, SMBUS_AutonomousModeConfTypeDef *sConfig)
{
  if (hsmbus->State == HAL_SMBUS_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_BUSY;

    /* Check the parameters */
    assert_param(IS_SMBUS_TRIG_SOURCE(hsmbus->Instance, sConfig->TriggerSelection));

    assert_param(IS_SMBUS_AUTO_MODE_TRG_POL(sConfig->TriggerPolarity));

    /* Disable the selected SMBUS peripheral to be able to configure AUTOCR */
    __HAL_SMBUS_DISABLE(hsmbus);

    /* SMBUSx AUTOCR Configuration */
    WRITE_REG(hsmbus->Instance->AUTOCR, (sConfig->TriggerState | ((sConfig->TriggerSelection) & I2C_AUTOCR_TRIGSEL_Msk) | sConfig->TriggerPolarity));

    /* Enable the selected SMBUS peripheral */
    __HAL_SMBUS_ENABLE(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Get Autonomous Mode configuration
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @param  sConfig Pointer to a SMBUS_AutonomousModeConfTypeDef structure that contains
  *                the configuration information of the autonomous mode for the specified SMBUSx peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUSEx_GetConfigAutonomousMode(SMBUS_HandleTypeDef *hsmbus, SMBUS_AutonomousModeConfTypeDef *sConfig)
{
  uint32_t autocr_tmp;

  autocr_tmp = hsmbus->Instance->AUTOCR;

  sConfig->TriggerState     = (autocr_tmp & I2C_AUTOCR_TRIGEN);
  if (IS_SMBUS_GRP2_INSTANCE(hsmbus->Instance))
  {
    sConfig->TriggerSelection = ((autocr_tmp & I2C_AUTOCR_TRIGSEL) | SMBUS_TRIG_GRP2);
  }
  else
  {
    sConfig->TriggerSelection = ((autocr_tmp & I2C_AUTOCR_TRIGSEL) | SMBUS_TRIG_GRP1);
  }
  sConfig->TriggerPolarity  = (autocr_tmp & I2C_AUTOCR_TRIGPOL);

  return HAL_OK;
}

/**
  * @brief  Clear Autonomous Mode configuration
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMBUSEx_ClearConfigAutonomousMode(SMBUS_HandleTypeDef *hsmbus)
{
  if (hsmbus->State == HAL_SMBUS_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_BUSY;

    /* Disable the selected SMBUS peripheral to be able to clear AUTOCR */
    __HAL_SMBUS_DISABLE(hsmbus);

    CLEAR_REG(hsmbus->Instance->AUTOCR);

    /* Enable the selected SMBUS peripheral */
    __HAL_SMBUS_ENABLE(hsmbus);

    hsmbus->State = HAL_SMBUS_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsmbus);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}
/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_SMBUS_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/