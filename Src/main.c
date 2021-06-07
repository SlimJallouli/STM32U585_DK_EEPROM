/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "m24xx.h"
#include "custom_bus.h"
#include <stdio.h>
/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENDPOINT     '1'
#define SSID         '2'
#define PSWD         '3'
#define DECURITY     '4'
#define SAVE         '5'
#define READ         '6'

#define WRITE_SIZE              M24256_PAGE_SIZE
#define READ_SIZE               WRITE_SIZE
#define ENDPOINT_SIZE           M24256_PAGE_SIZE
#define SSID_SIZE               M24256_PAGE_SIZE
#define PSWD_SIZE               M24256_PAGE_SIZE
#define WIFI_SECURITY_SIZE      1

#define ENDPOINT_ADDRESS        (0)
#define SSID_ADDRESS            (ENDPOINT_ADDRESS + ENDPOINT_SIZE)
#define PSWD_ADDRESS            (SSID_ADDRESS     + SSID_SIZE    )
#define WIFI_SECURITY_ADDRESS   (PSWD_ADDRESS     + PSWD_SIZE    )

#define STDOUT_UART_HANDLER     huart1
#define STDIN_UART_HANDLER      huart1
/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char endpoint       [ENDPOINT_SIZE] = {0};
char ssid           [SSID_SIZE    ] = {0};
char pswd           [PSWD_SIZE    ] = {0};
uint8_t wifi_security               = -1;

char saved_endpoint [ENDPOINT_SIZE] = {0};
char saved_ssid     [SSID_SIZE    ] = {0};
char saved_pswd     [PSWD_SIZE    ] = {0};
uint8_t saved_wifi_security         = -1;

char choice = '\0';

uint32_t err = 0;

M24_Object_t M24_pObj;
M24_IO_t     M24_pIO;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void flushRN(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */
  
  /* MCU Configuration--------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  M24_pIO.Init        = &BSP_I2C2_Init;
  M24_pIO.DeInit      = &BSP_I2C2_DeInit;
  M24_pIO.Address     = M24256_ADDRESS;
  M24_pIO.WriteReg    = &BSP_I2C2_WriteReg;
  M24_pIO.ReadReg     = &BSP_I2C2_ReadReg;
  M24_pIO.ReadReg16   = &BSP_I2C2_ReadReg16;
  M24_pIO.WriteReg16  = &BSP_I2C2_WriteReg16;
  M24_pIO.IsReady     = &BSP_I2C2_IsReady;
  M24_pIO.Delay       = &HAL_Delay;
  
  M24_RegisterBusIO(&M24_pObj, &M24_pIO);
  M24_i2c_Init(&M24_pObj);
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    printf("\r\n");
    printf("Enter %c to modify Endpoint\r\n"            , ENDPOINT    );
    printf("Enter %c to modify Wi-Fi SSID\r\n"          , SSID        );
    printf("Enter %c to modify Wi-Fi Password\r\n"      , PSWD        );
    printf("Enter %c to modify Wi-Fi Security\r\n"      , DECURITY    );
    printf("Enter %c to write to EEPROM\r\n"            , SAVE        );
    printf("Enter %c to read parameters from EEPROM\r\n", READ        );
    
    scanf("%c", &choice);
    
    printf("\r\n");
    
    switch(choice)
    {
    case ENDPOINT:
      printf("Enter Endpoint:\r\n");
      scanf("%s", endpoint);
      printf("Endpoint set to: %s\r\n", endpoint);
      flushRN();
      break;
      
    case SSID:
      printf("Enter Wi-Fi SSID:\r\n");
      scanf("%s", ssid);
      printf("WiFi SSID set to: %s\r\n", ssid);
      flushRN();
      break;
      
    case PSWD:
      printf("Enter Wi-Fi PSWD:\r\n");
      scanf("%s", pswd);
      printf("WiFi Password set to: %s\r\n", pswd);
      flushRN();
      break;
      
    case DECURITY:
      printf("0 = eWiFiSecurityOpen\r\n");
      printf("1 = eWiFiSecurityWEP\r\n");
      printf("2 = eWiFiSecurityWPA\r\n");
      printf("3 = eWiFiSecurityWPA2\r\n");
      printf("Enter Wi-Fi Security:\r\n");
      HAL_UART_Receive(&STDIN_UART_HANDLER, (uint8_t *)&wifi_security, 1, 0xFFFFFFFF);
      printf(" \n Wi-Fi Security set to: %c\r\n", wifi_security);
      flushRN();
      break;    
      
    case SAVE: //Write to EEPROM
      if (endpoint[0] != '\0')
      {
        printf("Writing DPS Endpoint to EEPROM: %s\r\n", endpoint);
        err = M24_i2c_WritePage(&M24_pObj, (uint8_t *)endpoint, ENDPOINT_ADDRESS, M24256_PAGE_SIZE, ENDPOINT_SIZE);
        HAL_Delay(M24256_PAGE_WRITE_TIME);
      }
      
      if (wifi_security - '0' <= 3)
      {
        printf("Writing Wi-Fi security to EEPROM: %c\r\n", wifi_security);
        err = M24_i2c_WriteData(&M24_pObj, (uint8_t *)&wifi_security, WIFI_SECURITY_ADDRESS, M24256_PAGE_SIZE, WIFI_SECURITY_SIZE);
        HAL_Delay(M24256_PAGE_WRITE_TIME);
      }
      
      if (ssid[0] != '\0')
      {
        printf("Writing WiFi SSID to EEPROM: %s\r\n", ssid);
        err = M24_i2c_WritePage(&M24_pObj, (uint8_t *)ssid, SSID_ADDRESS, M24256_PAGE_SIZE, SSID_SIZE);
        HAL_Delay(M24256_PAGE_WRITE_TIME);
      }
      
      if (pswd[0] != '\0')
      {
        printf("Writing WiFi Password to EEPROM: %s\r\n", pswd);
        err = M24_i2c_WritePage(&M24_pObj, (uint8_t *)pswd, PSWD_ADDRESS, M24256_PAGE_SIZE, PSWD_SIZE);
        HAL_Delay(M24256_PAGE_WRITE_TIME);
      }
      break;
      
    case READ: //read parameters from EEPROM
      err = M24_i2c_ReadData(&M24_pObj, (uint8_t *)saved_endpoint       , ENDPOINT_ADDRESS       , ENDPOINT_SIZE        );
      err = M24_i2c_ReadData(&M24_pObj, (uint8_t *)saved_ssid           , SSID_ADDRESS           , SSID_SIZE            );
      err = M24_i2c_ReadData(&M24_pObj, (uint8_t *)saved_pswd           , PSWD_ADDRESS           , PSWD_SIZE            );
      err = M24_i2c_ReadData(&M24_pObj, (uint8_t *)&saved_wifi_security , WIFI_SECURITY_ADDRESS  , WIFI_SECURITY_SIZE   );
      
      printf("Endpoint       : %s\r\n", saved_endpoint       );
      printf("Wi-Fi SSID     : %s\r\n", saved_ssid           );
      printf("Wi-Fi PSWD     : %s\r\n", saved_pswd           );
      printf("Wi-Fi Security : %c\r\n", saved_wifi_security  );      
      break;
      
    default:
      printf("%s\r\n", "Error Re-Enter Number");
    }
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
      |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  __HAL_RCC_PWR_CLK_DISABLE();
}

/**
* @brief ICACHE Initialization Function
* @param None
* @retval None
*/
static void MX_ICACHE_Init(void)
{
  
  /* USER CODE BEGIN ICACHE_Init 0 */
  
  /* USER CODE END ICACHE_Init 0 */
  
  /* USER CODE BEGIN ICACHE_Init 1 */
  
  /* USER CODE END ICACHE_Init 1 */
  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */
  
  /* USER CODE END ICACHE_Init 2 */
  
}

/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART1_UART_Init(void)
{
  
  /* USER CODE BEGIN USART1_Init 0 */
  
  /* USER CODE END USART1_Init 0 */
  
  /* USER CODE BEGIN USART1_Init 1 */
  
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  
  /* USER CODE END USART1_Init 2 */
  
}
/* Public functions ---------------------------------------------------------*/

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_RED_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, STMOD1_RST_Pin|WKUP_B_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MXCHIP_NSS_Pin|STMOD2_RST_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, STSAFE_RESET_Pin|MXCHIP_RESET_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STMOD1_SPI_NSS_GPIO_Port, STMOD1_SPI_NSS_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : STMOD1_17_Pin UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = STMOD1_17_Pin|UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /*Configure GPIO pins : MXCHIP_FLOW_Pin INT_LPS22HH_Pin */
  GPIO_InitStruct.Pin = MXCHIP_FLOW_Pin|INT_LPS22HH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD2_SPI_SCK_Pin STMOD2_SPI_MISO_Pin STMOD2_SPI_NSS_Pin */
  GPIO_InitStruct.Pin = STMOD2_SPI_SCK_Pin|STMOD2_SPI_MISO_Pin|STMOD2_SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  /*Configure GPIO pin : USB_UCPD_CC1_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_CC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_UCPD_CC1_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : UART3_SPI3_SEL_Pin UART2_SPI2_SEL_Pin VL53_xshut_Pin */
  GPIO_InitStruct.Pin = UART3_SPI3_SEL_Pin|UART2_SPI2_SEL_Pin|VL53_xshut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  
  /*Configure GPIO pin : STMOD2_SPI_MOSI_Pin */
  GPIO_InitStruct.Pin = STMOD2_SPI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI3;
  HAL_GPIO_Init(STMOD2_SPI_MOSI_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD1_18_Pin STMOD2_19_Pin STMOD2_18_Pin USB_IANA_Pin */
  GPIO_InitStruct.Pin = STMOD1_18_Pin|STMOD2_19_Pin|STMOD2_18_Pin|USB_IANA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD1_INT_Pin STMOD2_INT_Pin ISM330DLC_INT1_Pin */
  GPIO_InitStruct.Pin = STMOD1_INT_Pin|STMOD2_INT_Pin|ISM330DLC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /*Configure GPIO pins : UCPD_PWR_Pin USB_UCPD_CC2_Pin STMOD1_20_Pin */
  GPIO_InitStruct.Pin = UCPD_PWR_Pin|USB_UCPD_CC2_Pin|STMOD1_20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : STMOD1_TIM_Pin */
  GPIO_InitStruct.Pin = STMOD1_TIM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(STMOD1_TIM_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  
  /*Configure GPIO pin : SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_SCK_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD1_19_Pin VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = STMOD1_19_Pin|VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD1_RST_Pin WKUP_B_Pin */
  GPIO_InitStruct.Pin = STMOD1_RST_Pin|WKUP_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  /*Configure GPIO pin : STMOD2_TIM_Pin */
  GPIO_InitStruct.Pin = STMOD2_TIM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(STMOD2_TIM_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD2_17_Pin VLX_GPIO_Pin */
  GPIO_InitStruct.Pin = STMOD2_17_Pin|VLX_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  /*Configure GPIO pins : ADR_A0_Pin ARD_A1_Pin ARD_A2_Pin ARD_A3_Pin */
  GPIO_InitStruct.Pin = ADR_A0_Pin|ARD_A1_Pin|ARD_A2_Pin|ARD_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pins : ARD_A4_Pin STMOD1_ADC_Pin */
  GPIO_InitStruct.Pin = ARD_A4_Pin|STMOD1_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pin : STMOD2_ADC_Pin */
  GPIO_InitStruct.Pin = STMOD2_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STMOD2_ADC_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : MXCHIP_NOTIFY_Pin INT_IIS2MDC_Pin */
  GPIO_InitStruct.Pin = MXCHIP_NOTIFY_Pin|INT_IIS2MDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pin : ARD_A5_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_A5_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD1_SPI_MISO_Pin STMOD1_SPI_SCK_Pin STMOD1_SPI_MOSI_Pin */
  GPIO_InitStruct.Pin = STMOD1_SPI_MISO_Pin|STMOD1_SPI_SCK_Pin|STMOD1_SPI_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD2_UART_RTS_Pin STMOD2_UART_TX_Pin STMOD2_UART_RX_Pin STMOD2_USART_CTS_Pin */
  GPIO_InitStruct.Pin = STMOD2_UART_RTS_Pin|STMOD2_UART_TX_Pin|STMOD2_UART_RX_Pin|STMOD2_USART_CTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pins : MXCHIP_NSS_Pin STMOD2_RST_Pin */
  GPIO_InitStruct.Pin = MXCHIP_NSS_Pin|STMOD2_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STMOD1_UART_TX_Pin STMOD1_UART_RX_Pin */
  GPIO_InitStruct.Pin = STMOD1_UART_TX_Pin|STMOD1_UART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : STSAFE_RESET_Pin MXCHIP_RESET_Pin */
  GPIO_InitStruct.Pin = STSAFE_RESET_Pin|MXCHIP_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
  /*Configure GPIO pin : STMOD1_SPI_NSS_Pin */
  GPIO_InitStruct.Pin = STMOD1_SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STMOD1_SPI_NSS_GPIO_Port, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI14_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI14_IRQn);
  
  HAL_NVIC_SetPriority(EXTI15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_IRQn);
  
}

/* USER CODE BEGIN 4 */
void flushRN(void) 
{
  char cur;
  do {
    scanf("%c", &cur);
  } while (cur != '\r');//\r -> CR || \n -> CR + LF (TeraTerm)
}

#ifdef __ICCARM__
size_t __write(int handle, const unsigned char *buf, size_t bufSize)
#else
int _write(int handle, const unsigned char *buf, size_t bufSize)
#endif
{
  HAL_UART_Transmit(&STDOUT_UART_HANDLER,(uint8_t *)buf, bufSize, 0xFFFFFFFF);
  return bufSize;
}

#ifdef  __ICCARM__ 
size_t __read(int handle, unsigned char *buf, size_t bufSize)
#else
int _read(int handle, unsigned char *buf, size_t bufSize)
#endif
{
  int length = 0;
  int ch = 0;
  do
  {
    HAL_UART_Receive(&STDIN_UART_HANDLER, (uint8_t *)&ch, 1, 0xFFFFFFFF);
    *buf = ch;
    buf++;
    length++;
  }while((length < bufSize) && (*(buf-1) != '\r'));
  return length;
}
/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
