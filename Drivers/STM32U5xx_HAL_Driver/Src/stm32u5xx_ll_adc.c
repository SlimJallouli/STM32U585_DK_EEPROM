/**
  ******************************************************************************
  * @file    stm32u5xx_ll_adc.c
  * @author  MCD Application Team
  * @brief   ADC LL module driver
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
#if defined(USE_FULL_LL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_ll_adc.h"
#include "stm32u5xx_ll_bus.h"

#ifdef  USE_FULL_ASSERT
#include "stm32_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup STM32U5xx_LL_Driver
  * @{
  */

#if defined (ADC1) || defined (ADC2) || defined (ADC4)

/** @addtogroup ADC_LL ADC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup ADC_LL_Private_Constants
  * @{
  */

/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC peripheral HW delays are defined in ADC LL driver driver,   */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */
/* Note: ADC timeout values are defined here in CPU cycles to be independent  */
/*       of device clock setting.                                             */
/*       In user application, ADC timeout values should be defined with       */
/*       temporal values, in function of device clock settings.               */
/*       Highest ratio CPU clock frequency vs ADC clock frequency:            */
/*        - ADC clock from synchronous clock with AHB prescaler 512,          */
/*          APB prescaler 16, ADC prescaler 4.                                */
/*        - ADC clock from asynchronous clock (PLL) with prescaler 1,         */
/*          with highest ratio CPU clock frequency vs HSI clock frequency     */
/* Unit: CPU cycles.                                                          */
#define ADC_CLOCK_RATIO_VS_CPU_HIGHEST          (512UL * 16UL * 4UL)
#define ADC_TIMEOUT_DISABLE_CPU_CYCLES          (ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1UL)
#define ADC_TIMEOUT_STOP_CONVERSION_CPU_CYCLES  (ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1UL)

/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/

/** @addtogroup ADC_LL_Private_Macros
  * @{
  */

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* applicable for the twoinstances.                                           */
#define IS_LL_ADC_CLOCK(__CLOCK__)                                              \
  (   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV1)                                  \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV2)                               \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV4)                               \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV6)                               \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV8)                               \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV10)                              \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV12)                              \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV16)                              \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV32)                              \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV64)                              \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV128)                             \
      || ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV256)                             \
  )
/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC instance.                                                              */
#define IS_LL_ADC_RESOLUTION(__RESOLUTION__)                                    \
  (   ((__RESOLUTION__) == LL_ADC_RESOLUTION_14B)                               \
      || ((__RESOLUTION__) == LL_ADC_RESOLUTION_12B)                            \
      || ((__RESOLUTION__) == LL_ADC_RESOLUTION_10B)                            \
      || ((__RESOLUTION__) == LL_ADC_RESOLUTION_8B)                             \
      || ((__RESOLUTION__) == LL_ADC_RESOLUTION_6B)                             \
  )

#define IS_LL_ADC_DATA_ALIGN(__DATA_ALIGN__)                                    \
  (   ((__DATA_ALIGN__) == LL_ADC_DATA_ALIGN_RIGHT)                             \
      || ((__DATA_ALIGN__) == LL_ADC_DATA_ALIGN_LEFT)                           \
  )

#define IS_LL_ADC_LEFT_BIT_SHIFT(__LEFT_BIT_SHIFT__)                            \
  (   ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_NONE)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_1)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_2)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_3)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_4)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_5)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_6)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_7)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_8)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_9)                      \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_10)                     \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_11)                     \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_12)                     \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_13)                     \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_14)                     \
      || ((__LEFT_BIT_SHIFT__) == LL_ADC_LEFT_BIT_SHIFT_15)                     \
  )

#define IS_LL_ADC_LOW_POWER(__LOW_POWER__)                                      \
  (   ((__LOW_POWER__) == LL_ADC_LP_MODE_NONE)                                  \
      || ((__LOW_POWER__) == LL_ADC_LP_AUTOWAIT)                                \
  )

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC group regular                                                          */
#define IS_LL_ADC_REG_TRIG_SOURCE(__REG_TRIG_SOURCE__)                          \
  (   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_SOFTWARE)                       \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH1)                \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH2)                \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH3)                \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2)                \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_TRGO)               \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH4)                \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE11)             \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO)               \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO2)              \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO)               \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO2)              \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_TRGO)               \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_TRGO)               \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM6_TRGO)               \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM15_TRGO)              \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH4)                \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE15)             \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM1_CH1)              \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM2_CH1)              \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM3_CH1)              \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM4_OUT)              \
  )

#define IS_LL_ADC4_REG_TRIG_SOURCE(__REG_TRIG_SOURCE__)                         \
  (   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_SOFTWARE)                       \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO2_ADC4)         \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH4_ADC4)           \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_TRGO_ADC4)          \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM15_TRGO_ADC4)         \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM6_TRGO_ADC4)          \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM1_CH1_ADC4)         \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM3_CH2_ADC4)         \
      || ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE15_ADC4)        \
  )

#define IS_LL_ADC_REG_CONTINUOUS_MODE(__REG_CONTINUOUS_MODE__)                  \
  (   ((__REG_CONTINUOUS_MODE__) == LL_ADC_REG_CONV_SINGLE)                     \
      || ((__REG_CONTINUOUS_MODE__) == LL_ADC_REG_CONV_CONTINUOUS)              \
  )

#define IS_LL_ADC_REG_DATA_TRANSFER_MODE(__REG_DATA_TRANSFER_MODE__)            \
  (   ((__REG_DATA_TRANSFER_MODE__) == LL_ADC_REG_DR_TRANSFER)                  \
      || ((__REG_DATA_TRANSFER_MODE__) == LL_ADC_REG_DMA_TRANSFER_LIMITED)      \
      || ((__REG_DATA_TRANSFER_MODE__) == LL_ADC_REG_DMA_TRANSFER_UNLIMITED)    \
      || ((__REG_DATA_TRANSFER_MODE__) == LL_ADC_REG_MDF_TRANSFER)              \
  )
#define IS_LL_ADC_REG_DMA_TRANSFER(__REG_DMA_TRANSFER__)                        \
  (   ((__REG_DMA_TRANSFER__) == LL_ADC_REG_DMA_TRANSFER_NONE_ADC4)             \
      || ((__REG_DMA_TRANSFER__) == LL_ADC_REG_DMA_TRANSFER_LIMITED_ADC4)       \
      || ((__REG_DMA_TRANSFER__) == LL_ADC_REG_DMA_TRANSFER_UNLIMITED_ADC4)     \
  )

#define IS_LL_ADC_REG_OVR_DATA_BEHAVIOR(__REG_OVR_DATA_BEHAVIOR__)              \
  (   ((__REG_OVR_DATA_BEHAVIOR__) == LL_ADC_REG_OVR_DATA_PRESERVED)            \
      || ((__REG_OVR_DATA_BEHAVIOR__) == LL_ADC_REG_OVR_DATA_OVERWRITTEN)       \
  )

#define IS_LL_ADC_REG_SEQ_SCAN_LENGTH(__REG_SEQ_SCAN_LENGTH__)                  \
  (   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_DISABLE)                \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS)       \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS)      \
  )

#define IS_LL_ADC_REG_SEQ_MODE(__REG_SEQ_MODE__)                                \
  (   ((__REG_SEQ_MODE__) == LL_ADC_REG_SEQ_FIXED)                              \
      || ((__REG_SEQ_MODE__) == LL_ADC_REG_SEQ_CONFIGURABLE)                    \
  )

#define IS_LL_ADC4_REG_SEQ_SCAN_LENGTH(__REG_SEQ_SCAN_LENGTH__)                 \
  (   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_DISABLE)               \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_ENABLE_2RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_ENABLE_3RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_ENABLE_4RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_ENABLE_5RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_ENABLE_6RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_ENABLE_7RANKS)      \
      || ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC4_REG_SEQ_SCAN_ENABLE_8RANKS)      \
  )

#define IS_LL_ADC_REG_SEQ_SCAN_DISCONT_MODE(__REG_SEQ_DISCONT_MODE__)           \
  (   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_DISABLE)            \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_1RANK)           \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_2RANKS)          \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_3RANKS)          \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_4RANKS)          \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_5RANKS)          \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_6RANKS)          \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_7RANKS)          \
      || ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_8RANKS)          \
  )

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC group injected                                                         */

#define IS_LL_ADC_INJ_TRIG_SOURCE(__INJ_TRIG_SOURCE__)                          \
  (   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_SOFTWARE)                       \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO)               \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH4)                \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_TRGO)               \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_CH1)                \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH4)                \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_TRGO)               \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE15)             \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH4)                \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2)              \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO)               \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2)              \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH3)                \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_TRGO)               \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH1)                \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM6_TRGO)               \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM15_TRGO)              \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM1_CH2)              \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM2_CH2)              \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM3_CH1)              \
      || ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM4_OUT)              \
  )

#define IS_LL_ADC_INJ_TRIG_EXT_EDGE(__INJ_TRIG_EXT_EDGE__)                      \
  (   ((__INJ_TRIG_EXT_EDGE__) == LL_ADC_INJ_TRIG_EXT_RISING)                   \
      || ((__INJ_TRIG_EXT_EDGE__) == LL_ADC_INJ_TRIG_EXT_FALLING)               \
      || ((__INJ_TRIG_EXT_EDGE__) == LL_ADC_INJ_TRIG_EXT_RISINGFALLING)         \
  )

#define IS_LL_ADC_INJ_TRIG_AUTO(__INJ_TRIG_AUTO__)                              \
  (   ((__INJ_TRIG_AUTO__) == LL_ADC_INJ_TRIG_INDEPENDENT)                      \
      || ((__INJ_TRIG_AUTO__) == LL_ADC_INJ_TRIG_FROM_GRP_REGULAR)              \
  )

#define IS_LL_ADC_INJ_SEQ_SCAN_LENGTH(__INJ_SEQ_SCAN_LENGTH__)                  \
  (   ((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_DISABLE)                \
      || ((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS)       \
      || ((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS)       \
      || ((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS)       \
  )

#define IS_LL_ADC_INJ_SEQ_SCAN_DISCONT_MODE(__INJ_SEQ_DISCONT_MODE__)           \
  (   ((__INJ_SEQ_DISCONT_MODE__) == LL_ADC_INJ_SEQ_DISCONT_DISABLE)            \
      || ((__INJ_SEQ_DISCONT_MODE__) == LL_ADC_INJ_SEQ_DISCONT_1RANK)           \
  )

/**
  * @}
  */


/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_LL_Exported_Functions
  * @{
  */

/** @addtogroup ADC_LL_EF_Init
  * @{
  */
/**
  * @brief  De-initialize registers of all ADC instances belonging to
  *         the same ADC common instance to their default reset values.
  * @note   This function is performing a hard reset, using high level
  *         clock source RCC ADC reset.
  *         Caution: On this STM32 series, if several ADC instances are available
  *         on the selected device, RCC ADC reset will reset
  *         all ADC instances belonging to the common ADC instance.
  *         To de-initialize only 1 ADC instance, use
  *         function @ref LL_ADC_DeInit().
  * @param  ADCxy_COMMON ADC common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __LL_ADC_COMMON_INSTANCE() )
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC common registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus LL_ADC_CommonDeInit(ADC_Common_TypeDef *ADCxy_COMMON)
{
  /* Check the parameters */
  assert_param(IS_ADC_COMMON_INSTANCE(ADCxy_COMMON));

  if (ADCxy_COMMON == ADC12_COMMON)
  {
    /* Force reset of ADC clock (core clock) */
    LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC1);

    /* Release reset of ADC clock (core clock) */
    LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC1);
  }
  else   /*if ( ADCxy_COMMON == ADC4_COMMON)*/
  {
    /* Force reset of ADC clock (core clock) */
    LL_AHB3_GRP1_ForceReset(LL_AHB3_GRP1_PERIPH_ADC4);

    /* Release reset of ADC clock (core clock) */
    LL_AHB3_GRP1_ReleaseReset(LL_AHB3_GRP1_PERIPH_ADC4);
  }

  return SUCCESS;
}

/**
  * @brief  Initialize some features of ADC common parameters
  *         (all ADC instances belonging to the same ADC common instance)
  *         and multimode (for devices with several ADC instances available).
  * @note   The setting of ADC common parameters is conditioned to
  *         ADC instances state:
  *         All ADC instances belonging to the same ADC common instance
  *         must be disabled.
  * @param  ADCxy_COMMON ADC common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __LL_ADC_COMMON_INSTANCE() )
  * @param  ADC_CommonInitStruct Pointer to a @ref LL_ADC_CommonInitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC common registers are initialized
  *          - ERROR: ADC common registers are not initialized
  */
ErrorStatus LL_ADC_CommonInit(ADC_Common_TypeDef *ADCxy_COMMON, LL_ADC_CommonInitTypeDef *ADC_CommonInitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_COMMON_INSTANCE(ADCxy_COMMON));
  assert_param(IS_LL_ADC_CLOCK(ADC_CommonInitStruct->CommonClock));

  /* Note: Hardware constraint (refer to description of functions             */
  /*       "LL_ADC_SetCommonXXX()" and "LL_ADC_SetMultiXXX()"):               */
  /*       On this STM32 series, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       All ADC instances of the ADC common group must be disabled.        */
  if (__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(ADCxy_COMMON) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - common to several ADC                                               */
    /*    (all ADC instances belonging to the same ADC common instance)       */
    /*    - Set ADC clock (conversion clock)                                  */
    LL_ADC_SetCommonClock(ADCxy_COMMON, ADC_CommonInitStruct->CommonClock);
  }
  else
  {
    /* Initialization error: One or several ADC instances belonging to        */
    /* the same ADC common instance are not disabled.                         */
    status = ERROR;
  }

  return status;
}

/**
  * @brief  Set each @ref LL_ADC_CommonInitTypeDef field to default value.
  * @param  ADC_CommonInitStruct Pointer to a @ref LL_ADC_CommonInitTypeDef structure
  *                              whose fields will be set to default values.
  * @retval None
  */
void LL_ADC_CommonStructInit(LL_ADC_CommonInitTypeDef *ADC_CommonInitStruct)
{
  /* Set ADC_CommonInitStruct fields to default values */
  /* Set fields of ADC common */
  /* (all ADC instances belonging to the same ADC common instance) */
  ADC_CommonInitStruct->CommonClock = LL_ADC_CLOCK_ASYNC_DIV2;

}

/**
  * @brief  De-initialize registers of the selected ADC instance
  *         to their default reset values.
  * @note   To reset all ADC instances quickly (perform a hard reset),
  *         use function @ref LL_ADC_CommonDeInit().
  * @param  ADCx ADC instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR: ADC registers are not de-initialized
  */
ErrorStatus LL_ADC_DeInit(ADC_TypeDef *ADCx)
{
  ErrorStatus status = SUCCESS;

  __IO uint32_t timeout_cpu_cycles = 0UL;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));

  /* Disable ADC instance if not already disabled.                            */
  if (LL_ADC_IsEnabled(ADCx) == 1UL)
  {
    /* Set ADC group regular trigger source to SW start to ensure to not      */
    /* have an external trigger event occurring during the conversion stop    */
    /* ADC disable process.                                                   */
    LL_ADC_REG_SetTriggerSource(ADCx, LL_ADC_REG_TRIG_SOFTWARE);

    /* Stop potential ADC conversion on going on ADC group regular.           */
    if (LL_ADC_REG_IsConversionOngoing(ADCx) != 0UL)
    {
      if (LL_ADC_REG_IsStopConversionOngoing(ADCx) == 0UL)
      {
        LL_ADC_REG_StopConversion(ADCx);
      }
    }

    /* Wait for ADC conversions are effectively stopped                       */
    timeout_cpu_cycles = ADC_TIMEOUT_STOP_CONVERSION_CPU_CYCLES;
    if (ADCx != ADC4)  /* ADC1 or ADC2 */
    {
      /* Wait for ADC conversions are effectively stopped                       */
      timeout_cpu_cycles = ADC_TIMEOUT_STOP_CONVERSION_CPU_CYCLES;
      while ((LL_ADC_REG_IsStopConversionOngoing(ADCx)
              | LL_ADC_INJ_IsStopConversionOngoing(ADCx)) == 1UL)
      {
        timeout_cpu_cycles--;
        if (timeout_cpu_cycles == 0UL)
        {
          /* Time-out error */
          status = ERROR;
          break;
        }
      }

    }
    else
    {
      while (LL_ADC_REG_IsStopConversionOngoing(ADCx) == 1UL)
      {
        timeout_cpu_cycles--;
        if (timeout_cpu_cycles == 0UL)
        {
          /* Time-out error */
          status = ERROR;
        }
      }
    }
    /* Disable the ADC instance */
    LL_ADC_Disable(ADCx);

    /* Wait for ADC instance is effectively disabled */
    timeout_cpu_cycles = ADC_TIMEOUT_DISABLE_CPU_CYCLES;
    while (LL_ADC_IsDisableOngoing(ADCx) == 1UL)
    {
      timeout_cpu_cycles--;
      if (timeout_cpu_cycles == 0UL)
      {
        /* Time-out error */
        status = ERROR;
      }
    }
  }

  if (ADCx != ADC4) /* ADC1 or ADC2 */
  {
    CLEAR_BIT(ADCx->IER,
              (LL_ADC_IT_AWD3
               | LL_ADC_IT_AWD2
               | LL_ADC_IT_AWD1
               | LL_ADC_IT_JQOVF
               | LL_ADC_IT_OVR
               | LL_ADC_IT_JEOS
               | LL_ADC_IT_JEOC
               | LL_ADC_IT_EOS
               | LL_ADC_IT_EOC
               | LL_ADC_IT_EOSMP
               | LL_ADC_IT_ADRDY
              )
             );

    /* Reset register ISR */
    SET_BIT(ADCx->ISR,
            (LL_ADC_FLAG_AWD3
             | LL_ADC_FLAG_AWD2
             | LL_ADC_FLAG_AWD1
             | LL_ADC_FLAG_JQOVF
             | LL_ADC_FLAG_OVR
             | LL_ADC_FLAG_JEOS
             | LL_ADC_FLAG_JEOC
             | LL_ADC_FLAG_EOS
             | LL_ADC_FLAG_EOC
             | LL_ADC_FLAG_EOSMP
             | LL_ADC_FLAG_ADRDY
            )
           );

    /* Reset register CR */
    /* Bits ADC_CR_ADCAL, ADC_CR_ADSTP, ADC_CR_ADSTART are in access mode     */
    /* "read-set": no direct reset applicable.                                */
    CLEAR_BIT(ADCx->CR, ADC_CR_ADVREGEN);
    SET_BIT(ADCx->CR, ADC_CR_DEEPPWD);

    /* Reset register CFGR */
    CLEAR_BIT(ADCx->CFGR1, ADC_CFGR1_AWD1CH  | ADC_CFGR1_JAUTO   | ADC_CFGR1_JAWD1EN |
              ADC_CFGR1_AWD1EN  | ADC_CFGR1_AWD1SGL | ADC_CFGR1_JDISCEN |
              ADC_CFGR1_DISCNUM | ADC_CFGR1_DISCEN  | ADC_CFGR1_AUTDLY  |
              ADC_CFGR1_CONT    | ADC_CFGR1_OVRMOD  |
              ADC_CFGR1_EXTEN   | ADC_CFGR1_EXTSEL  |
              ADC_CFGR1_RES     | ADC_CFGR1_DMNGT);

    /* Reset register CFGR2 */
    CLEAR_BIT(ADCx->CFGR2, ADC_CFGR2_ROVSM  | ADC_CFGR2_TROVS   | ADC_CFGR2_OVSS |
              ADC_CFGR2_OVSR  | ADC_CFGR2_JOVSE | ADC_CFGR2_ROVSE);

    /* Reset register SMPR1 */
    CLEAR_BIT(ADCx->SMPR1,
              (ADC_SMPR1_SMP9 | ADC_SMPR1_SMP8 | ADC_SMPR1_SMP7
               | ADC_SMPR1_SMP6 | ADC_SMPR1_SMP5 | ADC_SMPR1_SMP4
               | ADC_SMPR1_SMP3 | ADC_SMPR1_SMP2 | ADC_SMPR1_SMP1)
             );

    /* Reset register SMPR2 */
    CLEAR_BIT(ADCx->SMPR2, ADC_SMPR2_SMP18 | ADC_SMPR2_SMP17 | ADC_SMPR2_SMP16 |
              ADC_SMPR2_SMP15 | ADC_SMPR2_SMP14 | ADC_SMPR2_SMP13 |
              ADC_SMPR2_SMP12 | ADC_SMPR2_SMP11 | ADC_SMPR2_SMP10);

    /* Reset register LTR1 and HTR1 */
    CLEAR_BIT(ADCx->LTR1, ADC_LTR_LT);
    SET_BIT(ADCx->HTR1, ADC_HTR_HT);

    /* Reset register LTR2 and HTR2*/
    CLEAR_BIT(ADCx->LTR2, ADC_LTR_LT);
    SET_BIT(ADCx->HTR2, ADC_HTR_HT);

    /* Reset register LTR3 and HTR3 */
    CLEAR_BIT(ADCx->LTR3, ADC_LTR_LT);
    SET_BIT(ADCx->HTR3, ADC_HTR_HT);

    /* Reset register SQR1 */
    CLEAR_BIT(ADCx->SQR1, ADC_SQR1_SQ4 | ADC_SQR1_SQ3 | ADC_SQR1_SQ2 |
              ADC_SQR1_SQ1 | ADC_SQR1_L);

    /* Reset register SQR2 */
    CLEAR_BIT(ADCx->SQR2, ADC_SQR2_SQ9 | ADC_SQR2_SQ8 | ADC_SQR2_SQ7 |
              ADC_SQR2_SQ6 | ADC_SQR2_SQ5);

    /* Reset register SQR3 */
    CLEAR_BIT(ADCx->SQR3, ADC_SQR3_SQ14 | ADC_SQR3_SQ13 | ADC_SQR3_SQ12 |
              ADC_SQR3_SQ11 | ADC_SQR3_SQ10);

    /* Reset register SQR4 */
    CLEAR_BIT(ADCx->SQR4, ADC_SQR4_SQ16 | ADC_SQR4_SQ15);

    /* Reset register JSQR */
    CLEAR_BIT(ADCx->JSQR,
              (ADC_JSQR_JL
               | ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN
               | ADC_JSQR_JSQ4    | ADC_JSQR_JSQ3
               | ADC_JSQR_JSQ2    | ADC_JSQR_JSQ1)
             );
    /* Reset register DR */
    /* bits in access mode read only, no direct reset applicable*/

    /* Reset register OFR1 */
    CLEAR_BIT(ADCx->OFR1, ADC_OFR1_SSAT | ADC_OFR1_OFFSET1_CH | ADC_OFR1_OFFSET1);
    /* Reset register OFR2 */
    CLEAR_BIT(ADCx->OFR2, ADC_OFR2_SSAT | ADC_OFR2_OFFSET2_CH | ADC_OFR2_OFFSET2);
    /* Reset register OFR3 */
    CLEAR_BIT(ADCx->OFR3, ADC_OFR3_SSAT | ADC_OFR3_OFFSET3_CH | ADC_OFR3_OFFSET3);
    /* Reset register OFR4 */
    CLEAR_BIT(ADCx->OFR4, ADC_OFR4_SSAT | ADC_OFR4_OFFSET4_CH | ADC_OFR4_OFFSET4);

    /* Reset register GCOMP */
    CLEAR_BIT(ADCx->GCOMP, ADC_GCOMP_GCOMP | ADC_GCOMP_GCOMPCOEFF);

    /* Reset registers JDR1, JDR2, JDR3, JDR4 */
    /* bits in access mode read only, no direct reset applicable*/

    /* Reset register AWD2CR */
    CLEAR_BIT(ADCx->AWD2CR, ADC_AWD2CR_AWD2CH);

    /* Reset register AWD3CR */
    CLEAR_BIT(ADCx->AWD3CR, ADC_AWD3CR_AWD3CH);

    /* Reset register DIFSEL */
    CLEAR_BIT(ADCx->DIFSEL, ADC_DIFSEL_DIFSEL);

    /* Reset register PCSEL */
    CLEAR_BIT(ADCx->PCSEL, ADC_PCSEL_PCSEL);

    /* Reset register CALFACT */
    CLEAR_BIT(ADCx->CALFACT, ADC_CALFACT_CAPTURE_COEF | ADC_CALFACT_LATCH_COEF);
  }
  else
  {
    /* Check whether ADC state is compliant with expected state */
    if (READ_BIT(ADCx->CR,
                 (ADC_CR_ADSTP | ADC_CR_ADSTART
                  | ADC_CR_ADDIS | ADC_CR_ADEN)
                )
        == 0UL)
    {
      /* ========== Reset ADC registers ========== */
      /* Reset register IER */
      CLEAR_BIT(ADCx->IER,
                (LL_ADC_IT_ADRDY
                 | LL_ADC_IT_EOC
                 | LL_ADC_IT_EOS
                 | LL_ADC_IT_OVR
                 | LL_ADC_IT_EOSMP
                 | LL_ADC_IT_AWD1
                 | LL_ADC_IT_AWD2
                 | LL_ADC_IT_AWD3
                 | LL_ADC_IT_EOCAL
                 | LL_ADC_IT_LDORDY
                )
               );

      /* Reset register ISR */
      SET_BIT(ADCx->ISR,
              (LL_ADC_FLAG_ADRDY
               | LL_ADC_FLAG_EOC
               | LL_ADC_FLAG_EOS
               | LL_ADC_FLAG_OVR
               | LL_ADC_FLAG_EOSMP
               | LL_ADC_FLAG_AWD1
               | LL_ADC_FLAG_AWD2
               | LL_ADC_FLAG_AWD3
               | LL_ADC_FLAG_EOCAL
               | LL_ADC_FLAG_LDORDY
              )
             );

      /* Reset register CR */
      /* Bits ADC_CR_ADCAL, ADC_CR_ADSTP, ADC_CR_ADSTART are in access mode     */
      /* "read-set": no direct reset applicable.                                */
      CLEAR_BIT(ADCx->CR, ADC_CR_ADVREGEN);

      /* Reset register CFGR1 */
      CLEAR_BIT(ADCx->CFGR1,
                (ADC_CFGR1_AWD1CH  | ADC_CFGR1_AWD1EN | ADC_CFGR1_AWD1SGL | ADC_CFGR1_DISCEN
                 | ADC4_CFGR1_WAIT   | ADC_CFGR1_CONT    | ADC_CFGR1_OVRMOD
                 | ADC_CFGR1_EXTEN   | ADC_CFGR1_EXTSEL | ADC4_CFGR1_ALIGN   | ADC_CFGR1_RES
                 | ADC4_CFGR1_SCANDIR | ADC4_CFGR1_DMACFG | ADC4_CFGR1_DMAEN)
               );

      /* Reset register CFGR2 */
      /* Note: Update of ADC clock mode is conditioned to ADC state disabled:   */
      /*       already done above.                                              */
      CLEAR_BIT(ADCx->CFGR2,
                (ADC_CFGR2_TROVS    | ADC_CFGR2_OVSS    | ADC4_CFGR2_OVSR
                 | ADC_CFGR2_ROVSE   | ADC4_CFGR2_LFTRIG)
               );

      /* Reset register SMPR */
      CLEAR_BIT(ADCx->SMPR1, ADC4_SMPR_SMP1 | ADC4_SMPR_SMP2 | ADC4_SMPR_SMPSEL);

      /* Reset register TR1 */
      MODIFY_REG(ADCx->AWD1TR, ADC_AWD1TR_HT1 | ADC_AWD1TR_LT1, ADC_AWD1TR_HT1);

      /* Reset register TR2 */
      MODIFY_REG(ADCx->AWD2TR, ADC_AWD2TR_HT2 | ADC_AWD2TR_LT2, ADC_AWD2TR_HT2);

      /* Reset register TR3 */
      MODIFY_REG(ADCx->AWD3TR, ADC_AWD3TR_HT3 | ADC_AWD3TR_LT3, ADC_AWD3TR_HT3);

      /* Reset register CHSELR */
      CLEAR_BIT(ADCx->CHSELR,
                (ADC_CHSELR_CHSEL23 | ADC_CHSELR_CHSEL22 | ADC_CHSELR_CHSEL21 | ADC_CHSELR_CHSEL20
                 | ADC_CHSELR_CHSEL19 | ADC_CHSELR_CHSEL18 | ADC_CHSELR_CHSEL17 | ADC_CHSELR_CHSEL16
                 | ADC_CHSELR_CHSEL15 | ADC_CHSELR_CHSEL14 | ADC_CHSELR_CHSEL13 | ADC_CHSELR_CHSEL12
                 | ADC_CHSELR_CHSEL11 | ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL9  | ADC_CHSELR_CHSEL8
                 | ADC_CHSELR_CHSEL7  | ADC_CHSELR_CHSEL6  | ADC_CHSELR_CHSEL5  | ADC_CHSELR_CHSEL4
                 | ADC_CHSELR_CHSEL3  | ADC_CHSELR_CHSEL2  | ADC_CHSELR_CHSEL1  | ADC_CHSELR_CHSEL0)
               );


      /* Reset register DR */
      /* bits in access mode read only, no direct reset applicable */

      /* Reset register CALFACT */
      CLEAR_BIT(ADCx->CALFACT, ADC4_CALFACT_CALFACT);

      /* Reset register CCR */
      CLEAR_BIT(ADC4_COMMON->CCR, ADC_CCR_VBATEN | ADC_CCR_VSENSEEN | ADC_CCR_VREFEN | ADC_CCR_PRESC);
    }
    else
    {
      /* ADC instance is in an unknown state */
      /* Need to performing a hard reset of ADC instance, using high level      */
      /* clock source RCC ADC reset.                                            */
      /* Caution: On this STM32 series, if several ADC instances are available   */
      /*          on the selected device, RCC ADC reset will reset              */
      /*          all ADC instances belonging to the common ADC instance.       */
      status = ERROR;
    }
  }

  return status;
}

/**
  * @brief  Initialize some features of ADC instance.
  * @note   These parameters have an impact on ADC scope: ADC instance.
  *         Affects both group regular and group injected (availability
  *         of ADC group injected depends on STM32 families).
  *         Refer to corresponding unitary functions into
  *         @ref ADC_LL_EF_Configuration_ADC_Instance .
  * @note   The setting of these parameters by function @ref LL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all STM32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @note   After using this function, some other features must be configured
  *         using LL unitary functions.
  *         The minimum configuration remaining to be done is:
  *          - Set ADC group regular or group injected sequencer:
  *            map channel on the selected sequencer rank.
  *            Refer to function @ref LL_ADC_REG_SetSequencerRanks().
  *          - Set ADC channel sampling time
  *            Refer to function LL_ADC_SetChannelSamplingTime();
  * @param  ADCx ADC instance
  * @param  ADC_InitStruct Pointer to a @ref LL_ADC_REG_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus LL_ADC_Init(ADC_TypeDef *ADCx, LL_ADC_InitTypeDef *ADC_InitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));

  assert_param(IS_LL_ADC_RESOLUTION(ADC_InitStruct->Resolution));
  assert_param(IS_LL_ADC_LOW_POWER(ADC_InitStruct->LowPowerMode));
  if (ADCx != ADC4) /* ADC1 or ADC2 */
  {
    assert_param(IS_LL_ADC_LEFT_BIT_SHIFT(ADC_InitStruct->LeftBitShift));
  }
  else
  {
    assert_param(IS_LL_ADC_DATA_ALIGN(ADC_InitStruct->DataAlignment));
  }

  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if (LL_ADC_IsEnabled(ADCx) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC instance                                                        */
    /*    - Set ADC data resolution                                           */
    /*    - Set ADC conversion data alignment                                 */
    /*    - Set ADC low power mode                                            */
    if (ADCx != ADC4) /* ADC1 or ADC2 */
    {
      MODIFY_REG(ADCx->CFGR1,
                 ADC_CFGR1_RES
                 | ADC4_CFGR1_WAIT
                 ,
                 ADC_InitStruct->Resolution
                 | ADC_InitStruct->LowPowerMode
                );
      MODIFY_REG(ADCx->CFGR2, ADC_CFGR2_LSHIFT, ADC_InitStruct->LeftBitShift);
    }
    else
    {
      MODIFY_REG(ADCx->CFGR1,
                 ADC_CFGR1_RES
                 | ADC4_CFGR1_ALIGN
                 | ADC4_CFGR1_WAIT
                 ,
                 __LL_ADC_RESOLUTION_ADC1_TO_ADC4(ADC_InitStruct->Resolution)
                 | ADC_InitStruct->DataAlignment
                 | ADC_InitStruct->LowPowerMode
                );
    }
  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref LL_ADC_InitTypeDef field to default value.
  * @param  ADCx ADC instance
  * @param  ADC_InitStruct Pointer to a @ref LL_ADC_InitTypeDef structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void LL_ADC_StructInit(ADC_TypeDef *ADCx, LL_ADC_InitTypeDef *ADC_InitStruct)
{
  /* Set ADC_InitStruct fields to default values */
  /* Set fields of ADC instance */
  ADC_InitStruct->LowPowerMode  = LL_ADC_LP_MODE_NONE;
  if (ADCx != ADC4) /* ADC1 or ADC2 */
  {
    ADC_InitStruct->Resolution    = LL_ADC_RESOLUTION_14B;
    ADC_InitStruct->LeftBitShift  = LL_ADC_LEFT_BIT_SHIFT_NONE;
  }
  else
  {
    ADC_InitStruct->Resolution    = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct->DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  }

}

/**
  * @brief  Initialize some features of ADC group regular.
  * @note   These parameters have an impact on ADC scope: ADC group regular.
  *         Refer to corresponding unitary functions into
  *         @ref ADC_LL_EF_Configuration_ADC_Group_Regular
  *         (functions with prefix "REG").
  * @note   The setting of these parameters by function @ref LL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all STM32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @note   After using this function, other features must be configured
  *         using LL unitary functions.
  *         The minimum configuration remaining to be done is:
  *          - Set ADC group regular or group injected sequencer:
  *            map channel on the selected sequencer rank.
  *            Refer to function @ref LL_ADC_REG_SetSequencerRanks().
  *          - Set ADC channel sampling time
  *            Refer to function LL_ADC_SetChannelSamplingTime();
  * @param  ADCx ADC instance
  * @param  ADC_REG_InitStruct Pointer to a @ref LL_ADC_REG_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus LL_ADC_REG_Init(ADC_TypeDef *ADCx, LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));
  if (ADCx == ADC1)
  {
    assert_param(IS_LL_ADC_REG_TRIG_SOURCE(ADC_REG_InitStruct->TriggerSource));
    assert_param(IS_LL_ADC_REG_SEQ_SCAN_LENGTH(ADC_REG_InitStruct->SequencerLength));
    assert_param(IS_LL_ADC_REG_DATA_TRANSFER_MODE(ADC_REG_InitStruct->DataTransferMode));
  }
  else
  {
    assert_param(IS_LL_ADC4_REG_TRIG_SOURCE(ADC_REG_InitStruct->TriggerSource));
    assert_param(IS_LL_ADC4_REG_SEQ_SCAN_LENGTH(ADC_REG_InitStruct->SequencerLength));
    assert_param(IS_LL_ADC_REG_DMA_TRANSFER(ADC_REG_InitStruct->DMATransfer));
    if ((LL_ADC_REG_GetSequencerConfigurable(ADCx) == LL_ADC_REG_SEQ_FIXED)
        || (ADC_REG_InitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE)
       )
    {
      assert_param(IS_LL_ADC_REG_SEQ_SCAN_DISCONT_MODE(ADC_REG_InitStruct->SequencerDiscont));

      /* ADC group regular continuous mode and discontinuous mode                 */
      /* can not be enabled simultenaeously                                       */
      assert_param((ADC_REG_InitStruct->ContinuousMode == LL_ADC_REG_CONV_SINGLE)
                   || (ADC_REG_InitStruct->SequencerDiscont == LL_ADC_REG_SEQ_DISCONT_DISABLE));
    }
  }
  if (ADC_REG_InitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE)
  {
    assert_param(IS_LL_ADC_REG_SEQ_SCAN_DISCONT_MODE(ADC_REG_InitStruct->SequencerDiscont));
  }
  assert_param(IS_LL_ADC_REG_CONTINUOUS_MODE(ADC_REG_InitStruct->ContinuousMode));
  assert_param(IS_LL_ADC_REG_OVR_DATA_BEHAVIOR(ADC_REG_InitStruct->Overrun));

  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if (LL_ADC_IsEnabled(ADCx) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC group regular                                                   */
    /*    - Set ADC group regular trigger source                              */
    /*    - Set ADC group regular sequencer length                            */
    /*    - Set ADC group regular sequencer discontinuous mode                */
    /*    - Set ADC group regular continuous mode                             */
    /*    - Set ADC group regular conversion data transfer: no transfer or    */
    /*      transfer by DMA, and DMA requests mode                            */
    /*    - Set ADC group regular overrun behavior                            */
    /* Note: On this STM32 series, ADC trigger edge is set when starting       */
    /*       ADC conversion.                                                  */
    /*       Refer to function @ref LL_ADC_REG_StartConversionExtTrig().      */
    if (ADCx != ADC4)  /* ADC1 or ADC2 */
    {
      if (ADC_REG_InitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE)
      {
        MODIFY_REG(ADCx->CFGR1,
                   ADC_CFGR1_EXTSEL
                   | ADC_CFGR1_EXTEN
                   | ADC_CFGR1_DISCEN
                   | ADC_CFGR1_DISCNUM
                   | ADC_CFGR1_CONT
                   | ADC_CFGR1_DMNGT
                   | ADC_CFGR1_OVRMOD
                   ,
                   ADC_REG_InitStruct->TriggerSource
                   | ADC_REG_InitStruct->SequencerDiscont
                   | ADC_REG_InitStruct->ContinuousMode
                   | ADC_REG_InitStruct->DataTransferMode
                   | ADC_REG_InitStruct->Overrun
                  );
      }
      else
      {
        MODIFY_REG(ADCx->CFGR1,
                   ADC_CFGR1_EXTSEL
                   | ADC_CFGR1_EXTEN
                   | ADC_CFGR1_DISCEN
                   | ADC_CFGR1_DISCNUM
                   | ADC_CFGR1_CONT
                   | ADC_CFGR1_DMNGT
                   | ADC_CFGR1_OVRMOD
                   ,
                   ADC_REG_InitStruct->TriggerSource
                   | LL_ADC_REG_SEQ_DISCONT_DISABLE
                   | ADC_REG_InitStruct->ContinuousMode
                   | ADC_REG_InitStruct->DataTransferMode
                   | ADC_REG_InitStruct->Overrun
                  );
      }
    }
    else
    {
      if ((LL_ADC_REG_GetSequencerConfigurable(ADCx) == LL_ADC_REG_SEQ_FIXED)
          || (ADC_REG_InitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE)
         )
      {
        MODIFY_REG(ADCx->CFGR1,
                   ADC_CFGR1_EXTSEL
                   | ADC_CFGR1_EXTEN
                   | ADC_CFGR1_DISCEN
                   | ADC_CFGR1_CONT
                   | ADC4_CFGR1_DMAEN
                   | ADC4_CFGR1_DMACFG
                   | ADC_CFGR1_OVRMOD
                   ,
                   ADC_REG_InitStruct->TriggerSource
                   | ADC_REG_InitStruct->SequencerDiscont
                   | ADC_REG_InitStruct->ContinuousMode
                   | ADC_REG_InitStruct->DMATransfer
                   | ADC_REG_InitStruct->Overrun
                  );
      }
      else
      {
        MODIFY_REG(ADCx->CFGR1,
                   ADC_CFGR1_EXTSEL
                   | ADC_CFGR1_EXTEN
                   | ADC_CFGR1_DISCEN
                   | ADC_CFGR1_CONT
                   | ADC4_CFGR1_DMAEN
                   | ADC4_CFGR1_DMACFG
                   | ADC_CFGR1_OVRMOD
                   ,
                   ADC_REG_InitStruct->TriggerSource
                   | LL_ADC_REG_SEQ_DISCONT_DISABLE
                   | ADC_REG_InitStruct->ContinuousMode
                   | ADC_REG_InitStruct->DMATransfer
                   | ADC_REG_InitStruct->Overrun
                  );
      }
    }

    /* Set ADC group regular sequencer length and scan direction */
    if (ADCx == ADC4)
    {
      if (LL_ADC_REG_GetSequencerConfigurable(ADCx) != LL_ADC_REG_SEQ_FIXED)
      {
        LL_ADC_REG_SetSequencerLength(ADCx, ADC_REG_InitStruct->SequencerLength);
      }
    }
    else
    {
      LL_ADC_REG_SetSequencerLength(ADCx, ADC_REG_InitStruct->SequencerLength);
    }
  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref LL_ADC_REG_InitTypeDef field to default value.
  * @param  ADCx ADC instance
  * @param  ADC_REG_InitStruct Pointer to a @ref LL_ADC_REG_InitTypeDef structure
  *                            whose fields will be set to default values.
  * @retval None
  */
void LL_ADC_REG_StructInit(ADC_TypeDef *ADCx, LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct)
{
  /* Set ADC_REG_InitStruct fields to default values */
  /* Set fields of ADC group regular */
  /* Note: On this STM32 series, ADC trigger edge is set when starting         */
  /*       ADC conversion.                                                    */
  /*       Refer to function @ref LL_ADC_REG_StartConversionExtTrig().        */
  ADC_REG_InitStruct->TriggerSource    = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct->SequencerLength  = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct->SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct->ContinuousMode   = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct->Overrun          = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  if (ADCx != ADC4) /* ADC1 or ADC2 */
  {
    ADC_REG_InitStruct->DataTransferMode = LL_ADC_REG_DR_TRANSFER;
  }
  else
  {
    ADC_REG_InitStruct->DMATransfer      = LL_ADC_REG_DMA_TRANSFER_NONE_ADC4;
  }
}

/**
  * @brief  Initialize some features of ADC group injected.
  * @note   These parameters have an impact on ADC scope: ADC group injected.
  *         Refer to corresponding unitary functions into
  *         @ref ADC_LL_EF_Configuration_ADC_Group_Regular
  *         (functions with prefix "INJ").
  * @note   The setting of these parameters by function @ref LL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         This condition is applied to all ADC features, for efficiency
  *         and compatibility over all STM32 families. However, the different
  *         features can be set under different ADC state conditions
  *         (setting possible with ADC enabled without conversion on going,
  *         ADC enabled with conversion on going, ...)
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  * @note   After using this function, other features must be configured
  *         using LL unitary functions.
  *         The minimum configuration remaining to be done is:
  *          - Set ADC group injected sequencer:
  *            map channel on the selected sequencer rank.
  *            Refer to function @ref LL_ADC_INJ_SetSequencerRanks().
  *          - Set ADC channel sampling time
  *            Refer to function LL_ADC_SetChannelSamplingTime();
  * @param  ADCx ADC instance
  * @param  ADC_INJ_InitStruct Pointer to a @ref LL_ADC_INJ_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized
  *          - ERROR: ADC registers are not initialized
  */
ErrorStatus LL_ADC_INJ_Init(ADC_TypeDef *ADCx, LL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct)
{
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));
  assert_param(IS_LL_ADC_INJ_TRIG_SOURCE(ADC_INJ_InitStruct->TriggerSource));
  assert_param(IS_LL_ADC_INJ_SEQ_SCAN_LENGTH(ADC_INJ_InitStruct->SequencerLength));
  if (ADC_INJ_InitStruct->SequencerLength != LL_ADC_INJ_SEQ_SCAN_DISABLE)
  {
    assert_param(IS_LL_ADC_INJ_SEQ_SCAN_DISCONT_MODE(ADC_INJ_InitStruct->SequencerDiscont));
  }
  assert_param(IS_LL_ADC_INJ_TRIG_AUTO(ADC_INJ_InitStruct->TrigAuto));

  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if (LL_ADC_IsEnabled(ADCx) == 0UL)
  {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC group injected                                                  */
    /*    - Set ADC group injected trigger source                             */
    /*    - Set ADC group injected sequencer length                           */
    /*    - Set ADC group injected sequencer discontinuous mode               */
    /*    - Set ADC group injected conversion trigger: independent or         */
    /*      from ADC group regular                                            */
    /* Note: On this STM32 series, ADC trigger edge is set when starting       */
    /*       ADC conversion.                                                  */
    /*       Refer to function @ref LL_ADC_INJ_StartConversionExtTrig().      */
    if (ADC_INJ_InitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE)
    {
      MODIFY_REG(ADCx->CFGR1,
                 ADC_CFGR1_JDISCEN
                 | ADC_CFGR1_JAUTO
                 ,
                 ADC_INJ_InitStruct->SequencerDiscont
                 | ADC_INJ_InitStruct->TrigAuto
                );
    }
    else
    {
      MODIFY_REG(ADCx->CFGR1,
                 ADC_CFGR1_JDISCEN
                 | ADC_CFGR1_JAUTO
                 ,
                 LL_ADC_REG_SEQ_DISCONT_DISABLE
                 | ADC_INJ_InitStruct->TrigAuto
                );
    }

    MODIFY_REG(ADCx->JSQR,
               ADC_JSQR_JEXTSEL
               | ADC_JSQR_JEXTEN
               | ADC_JSQR_JL
               ,
               ADC_INJ_InitStruct->TriggerSource
               | ADC_INJ_InitStruct->SequencerLength
              );
  }
  else
  {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
  * @brief  Set each @ref LL_ADC_INJ_InitTypeDef field to default value.
  * @param  ADC_INJ_InitStruct Pointer to a @ref LL_ADC_INJ_InitTypeDef structure
  *                            whose fields will be set to default values.
  * @retval None
  */
void LL_ADC_INJ_StructInit(LL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct)
{
  /* Set ADC_INJ_InitStruct fields to default values */
  /* Set fields of ADC group injected */
  ADC_INJ_InitStruct->TriggerSource    = LL_ADC_INJ_TRIG_SOFTWARE;
  ADC_INJ_InitStruct->SequencerLength  = LL_ADC_INJ_SEQ_SCAN_DISABLE;
  ADC_INJ_InitStruct->SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct->TrigAuto         = LL_ADC_INJ_TRIG_INDEPENDENT;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* ADC1 || ADC2 || ADC4 */

/**
  * @}
  */

#endif /* USE_FULL_LL_DRIVER */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/