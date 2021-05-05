/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   This file describe the main program.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        http://www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
/* 
 * This program is designed for a L475 IoT Dk board using HAL library. 
 * The program initialize the NFC component, then program the tags 
 * according to some NDEF classes available
 */
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "st25dv.h"
#include "custom_bus.h"

/** @addtogroup NFC_NDEF_Applications
  * @{
  */

/** @addtogroup NFC_Applications
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ENDPOINT_SIZE    64
#define IDSCOPE_SIZE     20
#define SSID_SIZE        20
#define PSWD_SIZE        20

#define ENDPOINT_ADDRESS 0
#define IDSCOPE_ADDRESS  (ENDPOINT_ADDRESS + ENDPOINT_SIZE)
#define SSID_ADDRESS     (IDSCOPE_ADDRESS  + IDSCOPE_SIZE )
#define PSWD_ADDRESS     (SSID_ADDRESS     + SSID_SIZE    )

#define STDOUT_UART_HANDLER huart1
#define STDIN_UART_HANDLER  huart1

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t NFCTAG_ID;

UART_HandleTypeDef huart1;

char endpoint      [ENDPOINT_SIZE] = {0};
char idscope       [IDSCOPE_SIZE ] = {0};
char ssid          [SSID_SIZE    ] = {0};
char pswd          [PSWD_SIZE    ] = {0};

char saved_endpoint[ENDPOINT_SIZE] = {0};
char saved_id      [IDSCOPE_SIZE ] = {0};
char saved_ssid    [SSID_SIZE    ] = {0};
char saved_pswd    [PSWD_SIZE    ] = {0};
char choice = '\0';

ST25DV_IO_t IO;
static ST25DV_Object_t NfcTagObj;

  
/* Private functions ---------------------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);
void flushRN(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{    
uint32_t err = 0;

  /* Reset of all peripherals, Initializes the Flash interface and the systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  
  /* Init of the ST25DV */
  /* Using the BUSIO in this project doesn't nean that I agree with it. I still don't like it at all. It is the worst thing ever */
  IO.Init         = BSP_I2C2_Init;
  IO.DeInit       = BSP_I2C2_DeInit;
  IO.IsReady      = BSP_I2C2_IsReady;
  IO.Read         = BSP_I2C2_ReadReg16;
  IO.Write        = (ST25DV_Write_Func)BSP_I2C2_WriteReg16;
  IO.GetTick      = HAL_GetTick;

  /* Using the BUSIO in this project doesn't nean that I agree with it. I don't like it at all. It is the worst thing ever */
  err = ST25DV_RegisterBusIO (&NfcTagObj, &IO);
  err = ST25DV_Init( &NfcTagObj );
  
  if(err)
  {
    printf("NFCTAG_Init error %d\r\n", err);
    __BKPT(0);
  }

  ST25DV_ReadID(&NfcTagObj, &NFCTAG_ID);
  
  printf("NFCTAG_ID: 0x%02X\r\n", NFCTAG_ID);

  while (1)
  {
    printf("%s\r\n", "Enter 1 to modify DPS Endpoint");
    printf("%s\r\n", "Enter 2 to modify ID Scope");
    printf("%s\r\n", "Enter 3 to modify SSID");
    printf("%s\r\n", "Enter 4 to modify Password");
    printf("%s\r\n", "Enter 5 to write to STSafe");
    printf("%s\r\n", "Enter 6 to read parameters from STSafe");
    scanf("%c", &choice);
    
    printf("\r\n");
    
    switch(choice)
    {
      case '1':
        printf("%s\r\n", "Enter DPS Endpoint:");
        scanf("%s", endpoint);
        printf("Endpoint set to: %s\r\n", endpoint);
        flushRN();
        break;

      case '2':
        printf("%s\r\n", "Enter ID Scope:");
        scanf("%s", idscope);
        printf("ID Scope set to: %s\r\n", idscope);
        flushRN();
        break;

      case '3':
        printf("%s\r\n", "Enter WIFI SSID:");
        scanf("%s", ssid);
        printf("WiFi SSID set to: %s\r\n", ssid);
        flushRN();
        break;
          
      case '4':
        printf("%s\r\n", "Enter WIFI PSWD:");
        scanf("%s", pswd);
        printf("WiFi Password set to: %s\r\n", pswd);
        flushRN();
        break;
          
      case '5': //Write to EEPROM
        if (endpoint[0] != '\0')
        {
          printf("Writing DPS Endpoint to STSafe: %s\r\n", endpoint);
           err = ST25DV_WriteData(&NfcTagObj, (uint8_t *)endpoint, ENDPOINT_ADDRESS, ENDPOINT_SIZE);
        }
        
        if (idscope[0] != '\0')
        {
          printf("Writing ID Scope to STSafe: %s\r\n", idscope);
          err = ST25DV_WriteData(&NfcTagObj, (uint8_t *)idscope, IDSCOPE_ADDRESS, IDSCOPE_SIZE);
        }
        
        if (ssid[0] != '\0')
        {
          printf("Writing WiFi SSID to STSafe: %s\r\n", ssid);
          err = ST25DV_WriteData(&NfcTagObj, (uint8_t *)ssid, SSID_ADDRESS, SSID_SIZE);
        }
        
        if (pswd[0] != '\0')
        {
          printf("Writing WiFi Password to STSafe: %s\r\n", pswd);
          err = ST25DV_WriteData(&NfcTagObj, (uint8_t *)pswd, PSWD_ADDRESS, PSWD_SIZE);
        }
        break;
          
      case '6': //read parameters from eeprom
        err = ST25DV_ReadData(&NfcTagObj, (uint8_t *)saved_endpoint, ENDPOINT_ADDRESS, ENDPOINT_SIZE);
        err = ST25DV_ReadData(&NfcTagObj, (uint8_t *)saved_id      , IDSCOPE_ADDRESS , IDSCOPE_SIZE );
        err = ST25DV_ReadData(&NfcTagObj, (uint8_t *)saved_ssid    , SSID_ADDRESS    , SSID_SIZE    );
        err = ST25DV_ReadData(&NfcTagObj, (uint8_t *)saved_pswd    , PSWD_ADDRESS    , PSWD_SIZE    );

        printf("Endpoint : %s\r\n", saved_endpoint);
        printf("ID Scope : %s\r\n", saved_id      );
        printf("WiFi SSID: %s\r\n", saved_ssid    );
        printf("WiFi PSWD: %s\r\n", saved_pswd    );
        break;
          
      default:
        printf("%s\r\n", "Error Re-Enter Number");
    }
    printf("\r\n");
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  return;
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
  huart1.Init.BaudRate = 9600;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : ST25DV_EH_Pin */
  GPIO_InitStruct.Pin = ST25DV_EH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ST25DV_EH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ST25DV_PO_Pin */
  GPIO_InitStruct.Pin = ST25DV_PO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ST25DV_PO_GPIO_Port, &GPIO_InitStruct);
}

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


/* Public functions ---------------------------------------------------------*/
void flushRN(void) 
{
  char cur;
  do {
  scanf("%c", &cur);
  } while (cur != '\r');//\r -> CR || \n -> CR + LF (TeraTerm)
}

#ifdef __ICCARM__
int __write(int file, char *ptr, int len)
#else
int _write(int file, char *ptr, int len)
#endif
{
  HAL_UART_Transmit(&STDOUT_UART_HANDLER,(uint8_t *)ptr, len, 0xFFFFFFFF);
  return len;
}

#ifdef  __ICCARM__ 
int   __read (int file, char *ptr, int len)
#else
int _read(int file, char *ptr, int len)
#endif
{
  int length = 0;
  int ch = 0;
  do
  {
    HAL_UART_Receive(&STDIN_UART_HANDLER, (uint8_t *)&ch, 1, 0xFFFFFFFF);
    *ptr = ch;
    ptr++;
    length++;
  }while((length < len) && (*(ptr-1) != '\r'));
  return length;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
