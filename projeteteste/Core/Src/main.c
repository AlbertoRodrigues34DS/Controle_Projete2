/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOTAOA (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==0)
#define BOTAOD (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==0)
#define BOTAOS (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==0)
#define BOTAOW (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==0)
#define BOTAOE (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==0)
#define BOTAOQ (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
}keyboardReportDes;

keyboardReportDes HIDkeyBoard={0,0,0,0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(BOTAOA)
	  {
		  HIDkeyBoard.MODIFIER = 0x02; // Print char in Capital
		  HIDkeyBoard.KEYCODE1 = 0x04; // Print 'A'
		  HAL_Delay(50);
		  USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
		  HIDkeyBoard.MODIFIER = 0x00;// Release Shift
		  HIDkeyBoard.KEYCODE1 = 0x00;// Release Key
	  }

	  if(BOTAOD)
	  {
		  HIDkeyBoard.MODIFIER = 0x02; // Print char in Capital
		  HIDkeyBoard.KEYCODE1 = 0x07; // Print 'D'
		  HAL_Delay(50);
		  USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
		  HIDkeyBoard.MODIFIER = 0x00;// Release Shift
		  HIDkeyBoard.KEYCODE1 = 0x00;// Release Key
	  }

	  if(BOTAOS)
	  {
		  HIDkeyBoard.MODIFIER = 0x02; // Print char in Capital
		  HIDkeyBoard.KEYCODE1 = 0x16; // Print 'S'
		  HAL_Delay(50);
		  USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
		  HIDkeyBoard.MODIFIER = 0x00;// Release Shift
		  HIDkeyBoard.KEYCODE1 = 0x00;// Release Key
	  }

	  if(BOTAOW)
	  {
		  HIDkeyBoard.MODIFIER = 0x02; // Print char in Capital
		  HIDkeyBoard.KEYCODE1 = 0x1A; // Print 'W'
		  HAL_Delay(50);
		  USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
		  HIDkeyBoard.MODIFIER = 0x00;// Release Shift
		  HIDkeyBoard.KEYCODE1 = 0x00;// Release Key
	  }

	  if(BOTAOE)
	  {
		  HIDkeyBoard.MODIFIER = 0x02; // Print char in Capital
		  HIDkeyBoard.KEYCODE1 = 0x08; // Print 'E'
		  HAL_Delay(50);
		  USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
		  HIDkeyBoard.MODIFIER = 0x00;// Release Shift
		  HIDkeyBoard.KEYCODE1 = 0x00;// Release Key
	  }
	  if(BOTAOQ)
	  {
		  HIDkeyBoard.MODIFIER = 0x02; // Print char in Capital
		  HIDkeyBoard.KEYCODE1 = 0x14; // Print 'Q'
		  HAL_Delay(50);
		  USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
		  HIDkeyBoard.MODIFIER = 0x00;// Release Shift
		  HIDkeyBoard.KEYCODE1 = 0x00;// Release Key
	  }






		/*
		HIDkeyBoard.MODIFIER = 0x02; // Print char in Capital
		HIDkeyBoard.KEYCODE1 = 0x04; // Print 'A'
		HIDkeyBoard.KEYCODE2 = 0x05; // Print 'B'
		HIDkeyBoard.KEYCODE3 = 0x06; // Print 'C'
		USBD_HID_SendReport(&hUsbDeviceFS, &HIDkeyBoard, sizeof(HIDkeyBoard));
		HAL_Delay(50);
		HIDkeyBoard.MODIFIER = 0x00; // Release Shift
		HIDkeyBoard.KEYCODE1 = 0x00;
		HIDkeyBoard.MODIFIER = 0x00; // Release Shift
		HIDkeyBoard.KEYCODE1 = 0x00; // Release Key
		HIDkeyBoard.



		*/
//
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
