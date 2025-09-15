/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t TxMailbox;
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
int messages[2] = {0x33, 0x37};
int motorMessage = false;
int motorSend = 0;
int PWM = 0;
int motorTemp = 0;
int contTemp = 0;
int RPM = 0;

int SOC = 99;
int highTherm = 99;
int packC = 99;
int ampHours = 99;
int packVS = 99;
int packVO = 99;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	switch(RxHeader.StdId)
	{
		case 0x114:
			//failsafes = RxData[0];
			//DTC1 = RxData[1];
			//DTC2 = RxData[2];
			highTherm = RxData[3];
			break;
		case 0x115:
			packC = RxData[0]/10;
			ampHours = RxData[1]/10;
			packVS = RxData[2]/100;
			packVO = RxData[3]/10;
			SOC = RxData[4]/2;
			break;
		case 0x73:
			switch (motorMessage)
			{
				case 0x33:
					PWM = RxData[0];
					motorTemp = RxData[2] + 80;
					contTemp = RxData[3];
					break;
				case 0x37:
					RPM = RxData[0] * 256;
					RPM += RxData[1];
					/*RPM *= 60;//to per hour
					RPM *= 2 * 3.141592653589 * 18.17;//to inches
					RPM/=12;//to feet
					RPM/=5280;//to miles*/

					RPM *= 0.1081112472;//combined into one step
					break;

			}
			motorMessage = 0;

	}
}

void motorRead(int data)
{
	for(int i = 0; i < 20 && motorMessage != 0; i++) HAL_Delay(25);
	motorMessage = data;

	TxHeader.DLC = 1;
	TxData[0] = 0x37;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x6B; // ID

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}


uint8_t segments[10] =
{
		0b00111111,
		0b00000110,
		0b01011011,
		0b01001111,
		0b01100110,
		0b01101101,
		0b01111101,
		0b00000111,
		0b01111111,
		0b01100111
};

int Dpins[3] = {DIO1_Pin, DIO2_Pin, DIO3_Pin};
int Cpins[3] = {Clk1_Pin, Clk2_Pin, Clk3_Pin};
GPIO_TypeDef* DGroup[3] = {GPIOB, GPIOB, GPIOA};
GPIO_TypeDef* CGroup[3] = {GPIOB, GPIOB, GPIOB};

void delay_us (int time)
{
	for (int i = 0; i < time; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			__asm__("nop");
		}
	}
}

void data(int set, int pin)
{
	HAL_GPIO_WritePin(DGroup[pin], Dpins[pin], set);
}

void clock(int set, int pin)
{
	HAL_GPIO_WritePin(CGroup[pin], Cpins[pin], set);
}

void start (int pin)
{
	clock(1, pin);
	data(1, pin);
	delay_us (2);
	data(0, pin);
}

void stop (int pin)
{
	clock(0, pin);
	delay_us (2);
	data(0, pin);
	delay_us (2);
	clock(1, pin);
	delay_us (2);
	data(1, pin);
}

void ack(int pin)
{
	clock(0, pin);
	delay_us(5);
	clock(1, pin);
	delay_us(2);
	clock(0, pin);
}

void dispByte(uint8_t d, int pin)
{
	for(int i = 0; i < 8; i++)
	{
		clock(0, pin);
		data(d%2, pin);
		delay_us(3);
		d = d >> 1;
		clock(1, pin);
		delay_us(3);
	}
	ack(pin);
}

void brightness(int pin)
{
	start(pin);
	dispByte(0x8A, pin);
	stop(pin);
}

void dispData(int index, uint8_t* d, int size, int pin)
{
	start(pin);
	dispByte(0x40, pin);
	stop(pin);

	start(pin);
	dispByte(192 + index, pin);
	for(int i = 0; i < size; i++) dispByte(d[i], pin);
	stop(pin);
	brightness(pin);
}

void dispNumber(int index, int rIndex, int n, int pin, int size, int colon)
{
	if(size == 0) for(int i = n; i > 0; i/=10) size++;

	uint8_t data[size];
	for(int i = size - 1; i >= 0; i--)
	{
		data[i] = segments[n%10];
		n/=10;
		if(colon != 0 && data[i] < 128) data[i] += 128;
	}

	if(rIndex != 0) index -= size - 1;

	dispData(index, data, size, pin);
}
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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_Delay(30);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  motorRead(messages[motorSend]);
	  motorSend++;
	  motorSend %= 2;

	  dispNumber(3, 1, RPM, 0, 2, 1);
	  dispNumber(0, 0, SOC, 0, 2, 1);
	  dispNumber(3, 1, motorTemp%100, 1, 2, 1);
	  dispNumber(0, 0, highTherm%100, 1, 2, 1);
	  dispNumber(3, 1, ampHours, 2, 2, 0);
	  dispNumber(0, 0, motorSend, 2, 2, 1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

   canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
   canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
   canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   canfilterconfig.FilterIdHigh = 0b10000<<5;
   canfilterconfig.FilterIdLow = 0;
   canfilterconfig.FilterMaskIdHigh = 0b10000<<5;
   canfilterconfig.FilterMaskIdLow = 0x0000;
   canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
   canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
   canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN1 (master can)

   HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIO1_Pin|Clk2_Pin|Clk3_Pin|DIO2_Pin
                          |Clk1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIO3_GPIO_Port, DIO3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIO1_Pin Clk2_Pin Clk3_Pin DIO2_Pin
                           Clk1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|Clk2_Pin|Clk3_Pin|DIO2_Pin
                          |Clk1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO3_Pin */
  GPIO_InitStruct.Pin = DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIO3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
