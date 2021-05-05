/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MXCHIP_SPI hspi2
#define console_uart huart1
#define MXCHIP_FLOW_EXTI_IRQn EXTI15_IRQn
#define MXCHIP_NOTIFY_EXTI_IRQn EXTI14_IRQn
#define STMOD1_SPI hspi1
#define STMOD_USART husart2
#define STMOD2_SPI hspi3
#define STMOD2_USART huart3
#define STMOD1_ADC hadc1
#define STMOD2_ADC hadc4
#define ARD_SPI hspi1
#define ENABLE_SENSOR 0
#define ENABLE_EEPROM 1
#define M24256_ADDRESS 0xAC
#define M24256_PAGE_SIZE 64
#define M24256_PAGE_WRITE_TIME 5
#define M24256_BYTE_WRITE_TIME 5
#define STMOD1_17_Pin GPIO_PIN_2
#define STMOD1_17_GPIO_Port GPIOE
#define MXCHIP_FLOW_Pin GPIO_PIN_15
#define MXCHIP_FLOW_GPIO_Port GPIOG
#define MXCHIP_FLOW_EXTI_IRQn EXTI15_IRQn
#define STMOD2_SPI_SCK_Pin GPIO_PIN_9
#define STMOD2_SPI_SCK_GPIO_Port GPIOG
#define USB_UCPD_CC1_Pin GPIO_PIN_15
#define USB_UCPD_CC1_GPIO_Port GPIOA
#define UART3_SPI3_SEL_Pin GPIO_PIN_15
#define UART3_SPI3_SEL_GPIO_Port GPIOH
#define STMOD2_SPI_MOSI_Pin GPIO_PIN_6
#define STMOD2_SPI_MOSI_GPIO_Port GPIOD
#define STMOD1_18_Pin GPIO_PIN_0
#define STMOD1_18_GPIO_Port GPIOD
#define STMOD1_INT_Pin GPIO_PIN_4
#define STMOD1_INT_GPIO_Port GPIOE
#define UCPD_PWR_Pin GPIO_PIN_5
#define UCPD_PWR_GPIO_Port GPIOB
#define STMOD2_SPI_MISO_Pin GPIO_PIN_10
#define STMOD2_SPI_MISO_GPIO_Port GPIOG
#define UART2_SPI2_SEL_Pin GPIO_PIN_13
#define UART2_SPI2_SEL_GPIO_Port GPIOH
#define STMOD1_TIM_Pin GPIO_PIN_5
#define STMOD1_TIM_GPIO_Port GPIOE
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define STMOD2_INT_Pin GPIO_PIN_6
#define STMOD2_INT_GPIO_Port GPIOE
#define STMOD2_SPI_NSS_Pin GPIO_PIN_12
#define STMOD2_SPI_NSS_GPIO_Port GPIOG
#define STMOD2_19_Pin GPIO_PIN_5
#define STMOD2_19_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_6
#define LED_RED_GPIO_Port GPIOH
#define SPI2_SCK_Pin GPIO_PIN_1
#define SPI2_SCK_GPIO_Port GPIOD
#define STMOD2_18_Pin GPIO_PIN_2
#define STMOD2_18_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOH
#define STMOD1_19_Pin GPIO_PIN_5
#define STMOD1_19_GPIO_Port GPIOF
#define STMOD1_RST_Pin GPIO_PIN_7
#define STMOD1_RST_GPIO_Port GPIOG
#define STMOD2_TIM_Pin GPIO_PIN_9
#define STMOD2_TIM_GPIO_Port GPIOC
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define STMOD2_17_Pin GPIO_PIN_1
#define STMOD2_17_GPIO_Port GPIOG
#define WKUP_B_Pin GPIO_PIN_6
#define WKUP_B_GPIO_Port GPIOG
#define VL53_xshut_Pin GPIO_PIN_1
#define VL53_xshut_GPIO_Port GPIOH
#define ADR_A0_Pin GPIO_PIN_0
#define ADR_A0_GPIO_Port GPIOC
#define ARD_A1_Pin GPIO_PIN_2
#define ARD_A1_GPIO_Port GPIOC
#define ARD_A4_Pin GPIO_PIN_7
#define ARD_A4_GPIO_Port GPIOA
#define STMOD2_ADC_Pin GPIO_PIN_0
#define STMOD2_ADC_GPIO_Port GPIOG
#define VLX_GPIO_Pin GPIO_PIN_5
#define VLX_GPIO_GPIO_Port GPIOG
#define MXCHIP_NOTIFY_Pin GPIO_PIN_14
#define MXCHIP_NOTIFY_GPIO_Port GPIOD
#define MXCHIP_NOTIFY_EXTI_IRQn EXTI14_IRQn
#define STMOD1_ADC_Pin GPIO_PIN_5
#define STMOD1_ADC_GPIO_Port GPIOA
#define ARD_A5_Pin GPIO_PIN_0
#define ARD_A5_GPIO_Port GPIOB
#define UCPD_FLT_Pin GPIO_PIN_8
#define UCPD_FLT_GPIO_Port GPIOE
#define STMOD1_SPI_MISO_Pin GPIO_PIN_14
#define STMOD1_SPI_MISO_GPIO_Port GPIOE
#define STMOD2_UART_RTS_Pin GPIO_PIN_12
#define STMOD2_UART_RTS_GPIO_Port GPIOD
#define INT_IIS2MDC_Pin GPIO_PIN_10
#define INT_IIS2MDC_GPIO_Port GPIOD
#define USB_IANA_Pin GPIO_PIN_13
#define USB_IANA_GPIO_Port GPIOD
#define INT_LPS22HH_Pin GPIO_PIN_2
#define INT_LPS22HH_GPIO_Port GPIOG
#define ARD_A2_Pin GPIO_PIN_4
#define ARD_A2_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_14
#define VBUS_SENSE_GPIO_Port GPIOF
#define STMOD1_SPI_SCK_Pin GPIO_PIN_13
#define STMOD1_SPI_SCK_GPIO_Port GPIOE
#define MXCHIP_NSS_Pin GPIO_PIN_12
#define MXCHIP_NSS_GPIO_Port GPIOB
#define USB_UCPD_CC2_Pin GPIO_PIN_15
#define USB_UCPD_CC2_GPIO_Port GPIOB
#define STMOD2_UART_TX_Pin GPIO_PIN_8
#define STMOD2_UART_TX_GPIO_Port GPIOD
#define STMOD2_UART_RX_Pin GPIO_PIN_9
#define STMOD2_UART_RX_GPIO_Port GPIOD
#define STMOD1_UART_TX_Pin GPIO_PIN_2
#define STMOD1_UART_TX_GPIO_Port GPIOA
#define ARD_A3_Pin GPIO_PIN_5
#define ARD_A3_GPIO_Port GPIOC
#define STSAFE_RESET_Pin GPIO_PIN_11
#define STSAFE_RESET_GPIO_Port GPIOF
#define ISM330DLC_INT1_Pin GPIO_PIN_11
#define ISM330DLC_INT1_GPIO_Port GPIOE
#define STMOD1_SPI_MOSI_Pin GPIO_PIN_15
#define STMOD1_SPI_MOSI_GPIO_Port GPIOE
#define STMOD1_20_Pin GPIO_PIN_14
#define STMOD1_20_GPIO_Port GPIOB
#define STMOD2_USART_CTS_Pin GPIO_PIN_11
#define STMOD2_USART_CTS_GPIO_Port GPIOD
#define STMOD1_SPI_NSS_Pin GPIO_PIN_4
#define STMOD1_SPI_NSS_GPIO_Port GPIOA
#define STMOD1_UART_RX_Pin GPIO_PIN_3
#define STMOD1_UART_RX_GPIO_Port GPIOA
#define MXCHIP_RESET_Pin GPIO_PIN_15
#define MXCHIP_RESET_GPIO_Port GPIOF
#define STMOD2_RST_Pin GPIO_PIN_13
#define STMOD2_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
