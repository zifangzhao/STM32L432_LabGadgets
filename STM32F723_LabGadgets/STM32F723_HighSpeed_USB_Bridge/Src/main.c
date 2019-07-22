/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dataMGR.h"
#include "CE32_ioncom.h"
#include "usbd_cdc_if.h"

#define CDC_BUF_SIZE 0x2000
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
int Bytes;
int Nfail;
int retries;
int WaitCyc;
uint8_t data_buf_RX1[BUF_SIZE];		// This records @ 20kHz x 32CH
uint8_t data_buf_RX2[BUF_SIZE];		// This records @ 20kHz x 32CH
uint8_t data_buf_RX3[BUF_SIZE];		// This records @ 20kHz x 32CH
uint8_t data_buf_RX4[BUF_SIZE];		// This records @ 20kHz x 32CH
uint8_t data_buf_TX[BUF_SIZE];		// This records @ 20kHz x 32CH
uint8_t data_buf_RX_CDC[CDC_BUF_SIZE];
uint16_t misc_DigiSig;
uint8_t rec_cnt;
uint32_t log_cnt;
uint32_t test;
uint32_t LED_CNT;
uint32_t logIdx;	//Index of log in the CE32_systemLog struct
uint32_t filePTR;	//recording pointer location (in 512B unit,real filelen=512*x)
uint32_t updHeader;
dataMGR MGR_TX;
dataMGR MGR_RX1;
dataMGR MGR_RX2;
dataMGR MGR_RX3;
dataMGR MGR_RX4;
dataMGR MGR_CDC;
dataMGR MGR_prev;
dataMGR MGR_misc;
dataMGR MGR_cmd;
uint8_t HD32_sd_temp;

//dataMGR PMGR;
int freq;

const uint16_t RX_size=1000;

struct CE32_IONCOM_Handle IC_handle1;
struct CE32_IONCOM_Handle IC_handle2;
struct CE32_IONCOM_Handle IC_handle3;
struct CE32_IONCOM_Handle IC_handle4;

extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_UART7_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM14_Init(void);
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
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//Must comment MX_USB_Device_init in the initialize code
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  //MX_USB_DEVICE_Init();
  MX_UART7_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_10,GPIO_PIN_SET);//switch Pmode
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_SET);//switch Pmode
	
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_8,GPIO_PIN_SET);//LED0
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);//LED1
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);//LED2
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);//LED3
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_8,GPIO_PIN_RESET);//LED0
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);//LED1
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_RESET);//LED2
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET);//LED3
	
	dataMGR_init(&MGR_TX,(char*) data_buf_TX,sizeof(data_buf_TX));					//FIFO setup 
	dataMGR_init(&MGR_RX1,(char*) data_buf_RX1,sizeof(data_buf_RX1));					//RX FIFO setup 
	dataMGR_init(&MGR_RX2,(char*) data_buf_RX2,sizeof(data_buf_RX2));					//RX FIFO setup 
	dataMGR_init(&MGR_RX3,(char*) data_buf_RX3,sizeof(data_buf_RX3));					//RX FIFO setup 
	dataMGR_init(&MGR_RX4,(char*) data_buf_RX4,sizeof(data_buf_RX4));					//RX FIFO setup 
	dataMGR_init(&MGR_CDC,(char*) data_buf_RX_CDC,sizeof(data_buf_RX_CDC));					//RX FIFO setup 
	
	IC_handle1.DMA_TotalBanks=4;
	IC_handle1.DMA_TransSize=BUF_SIZE/IC_handle1.DMA_TotalBanks;
	IC_handle1.huart=&huart7;
	IC_handle1.IRQn=UART7_IRQn;
	IC_handle1.huart->hdmarx->Instance=DMA1_Stream3;
	IC_handle1.config|=(IONCOM_CONFIG_RXDMA);
	CE32_Ioncom_Init(&IC_handle1,(char*)data_buf_RX1,sizeof(data_buf_RX1),(char*)data_buf_TX,(uint16_t)sizeof(data_buf_TX));
	UART_IONCOM_DMA_Init(&IC_handle1);
	UART_DISABLE(IC_handle1.huart->Instance);					//Disable UART temporarly to avoid interferece with initialization
	IC_handle1.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle1.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
	IC_handle1.huart->hdmarx->Instance->NDTR = IC_handle1.RX_MGR.dataSize;
	
	
	IC_handle2.DMA_TotalBanks=4;
	IC_handle2.DMA_TransSize=BUF_SIZE/IC_handle2.DMA_TotalBanks;
	IC_handle2.huart=&huart3;
	IC_handle2.IRQn=USART3_IRQn;
	IC_handle2.huart->hdmarx->Instance=DMA1_Stream1;
	IC_handle2.config|=(IONCOM_CONFIG_RXDMA);
	CE32_Ioncom_Init(&IC_handle2,(char*)data_buf_RX2,sizeof(data_buf_RX2),(char*)data_buf_TX,(uint16_t)sizeof(data_buf_TX));
	UART_IONCOM_DMA_Init(&IC_handle2);
	UART_DISABLE(IC_handle2.huart->Instance);					//Disable UART temporarly to avoid interferece with initialization
	IC_handle2.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle2.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
	IC_handle2.huart->hdmarx->Instance->NDTR = IC_handle2.RX_MGR.dataSize;
	
	IC_handle3.DMA_TotalBanks=4;
	IC_handle3.DMA_TransSize=BUF_SIZE/IC_handle3.DMA_TotalBanks;
	IC_handle3.huart=&huart4;
	IC_handle3.IRQn=UART4_IRQn;
	IC_handle3.huart->hdmarx->Instance=DMA1_Stream2;
	IC_handle3.config|=(IONCOM_CONFIG_RXDMA);
	CE32_Ioncom_Init(&IC_handle3,(char*)data_buf_RX3,sizeof(data_buf_RX3),(char*)data_buf_TX,(uint16_t)sizeof(data_buf_TX));
	UART_IONCOM_DMA_Init(&IC_handle3);
	UART_DISABLE(IC_handle3.huart->Instance);					//Disable UART temporarly to avoid interferece with initialization
	IC_handle3.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle3.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
	IC_handle3.huart->hdmarx->Instance->NDTR = IC_handle3.RX_MGR.dataSize;
	
	IC_handle4.DMA_TotalBanks=4;
	IC_handle4.DMA_TransSize=BUF_SIZE/IC_handle4.DMA_TotalBanks;
	IC_handle4.huart=&huart2;
	IC_handle4.IRQn=USART2_IRQn;
	IC_handle4.huart->hdmarx->Instance=DMA1_Stream5;
	IC_handle4.config|=(IONCOM_CONFIG_RXDMA);
	CE32_Ioncom_Init(&IC_handle4,(char*)data_buf_RX4,sizeof(data_buf_RX4),(char*)data_buf_TX,(uint16_t)sizeof(data_buf_TX));
	UART_IONCOM_DMA_Init(&IC_handle4);
	UART_DISABLE(IC_handle4.huart->Instance);					//Disable UART temporarly to avoid interferece with initialization
	IC_handle4.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle4.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
	IC_handle4.huart->hdmarx->Instance->NDTR = IC_handle4.RX_MGR.dataSize;
	//UART7->CR1|=USART_CR1_RXNEIE;  //Enable RX interrput to receive commands

	MX_USB_DEVICE_Init();
	
	TIM14->CR1|=TIM_CR1_CEN;	//Enable timer
	TIM14->DIER|=TIM_DIER_UIE;//Enable update interrupt
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//DIrect mode
		if(IC_handle1.RX_MGR.bufferUsed[0]>=148)
		{
			if((uint8_t)dataMGR_deQueue_byte(&IC_handle1.RX_MGR,0)==0x66)
			{
				if((uint8_t)dataMGR_deQueue_byte(&IC_handle1.RX_MGR,0)==0x55)
				{
					if((uint8_t)dataMGR_deQueue_byte(&IC_handle1.RX_MGR,0)==0x5d)
					{
						if((uint8_t)dataMGR_deQueue_byte(&IC_handle1.RX_MGR,0)==0xd5)
						{
							HAL_GPIO_WritePin(GPIOF,GPIO_PIN_8,GPIO_PIN_SET);
							dataMGR_enQueue_byte(&MGR_CDC,0x66);
							dataMGR_enQueue_byte(&MGR_CDC,0x55);
							dataMGR_enQueue_byte(&MGR_CDC,0x5d);
							dataMGR_enQueue_byte(&MGR_CDC,0x00);
							for(int i=0;i<144;i++)
							{
								while(IC_handle1.RX_MGR.bufferUsed[0]<=0)
								{
								}
								//dataMGR_enQueue_byte(&MGR_CDC,0xAA);
								uint8_t temp=(uint8_t)dataMGR_deQueue_byte(&IC_handle1.RX_MGR,0);
//								if(temp==0)
//								{
//									__nop();
//								}
								dataMGR_enQueue_byte(&MGR_CDC,temp);
							}
							
							//break;
						}
					}
				}
			}
		}

		if(IC_handle2.RX_MGR.bufferUsed[0]>=148)
		{
			if((uint8_t)dataMGR_deQueue_byte(&IC_handle2.RX_MGR,0)==0x66)
			{
				if((uint8_t)dataMGR_deQueue_byte(&IC_handle2.RX_MGR,0)==0x55)
				{
					if((uint8_t)dataMGR_deQueue_byte(&IC_handle2.RX_MGR,0)==0x5d)
					{
						if((uint8_t)dataMGR_deQueue_byte(&IC_handle2.RX_MGR,0)==0xd5)
						{
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
							dataMGR_enQueue_byte(&MGR_CDC,0x66);
							dataMGR_enQueue_byte(&MGR_CDC,0x55);
							dataMGR_enQueue_byte(&MGR_CDC,0x5d);
							dataMGR_enQueue_byte(&MGR_CDC,0x01);
							for(int i=0;i<144;i++)
							{
								while(IC_handle2.RX_MGR.bufferUsed[0]<=0)
								{
								}
								dataMGR_enQueue_byte(&MGR_CDC,(uint8_t)dataMGR_deQueue_byte(&IC_handle2.RX_MGR,0));
							}
							//break;
							
						}
					}
				}
			}
		}

		
		if(IC_handle3.RX_MGR.bufferUsed[0]>=148)
		{
			if((uint8_t)dataMGR_deQueue_byte(&IC_handle3.RX_MGR,0)==0x66)
			{
				if((uint8_t)dataMGR_deQueue_byte(&IC_handle3.RX_MGR,0)==0x55)
				{
					if((uint8_t)dataMGR_deQueue_byte(&IC_handle3.RX_MGR,0)==0x5d)
					{
						if((uint8_t)dataMGR_deQueue_byte(&IC_handle3.RX_MGR,0)==0xd5)
						{
							HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);
							dataMGR_enQueue_byte(&MGR_CDC,0x66);
							dataMGR_enQueue_byte(&MGR_CDC,0x55);
							dataMGR_enQueue_byte(&MGR_CDC,0x5d);
							dataMGR_enQueue_byte(&MGR_CDC,0x02);
							for(int i=0;i<144;i++)
							{
								while(IC_handle3.RX_MGR.bufferUsed[0]<=0)
								{
								}
								dataMGR_enQueue_byte(&MGR_CDC,(uint8_t)dataMGR_deQueue_byte(&IC_handle3.RX_MGR,0));
							}
							
							//break;
						}
					}
				}
			}
		}

		if(IC_handle4.RX_MGR.bufferUsed[0]>=148)
		{
			if((uint8_t)dataMGR_deQueue_byte(&IC_handle4.RX_MGR,0)==0x66)
			{
				if((uint8_t)dataMGR_deQueue_byte(&IC_handle4.RX_MGR,0)==0x55)
				{
					if((uint8_t)dataMGR_deQueue_byte(&IC_handle4.RX_MGR,0)==0x5d)
					{
						if((uint8_t)dataMGR_deQueue_byte(&IC_handle4.RX_MGR,0)==0xd5)
						{
							HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);
							dataMGR_enQueue_byte(&MGR_CDC,0x66);
							dataMGR_enQueue_byte(&MGR_CDC,0x55);
							dataMGR_enQueue_byte(&MGR_CDC,0x5d);
							dataMGR_enQueue_byte(&MGR_CDC,0x03);
							for(int i=0;i<144;i++)
							{
								while(IC_handle4.RX_MGR.bufferUsed[0]<=0)
								{
								}
								dataMGR_enQueue_byte(&MGR_CDC,(uint8_t)dataMGR_deQueue_byte(&IC_handle4.RX_MGR,0));
							}
							
							//break;
						}
					}
				}
			}
		}
		
		if(MGR_CDC.bufferUsed[0]>=CDC_BUF_SIZE/4)
		{
			int ByteToSend=CDC_BUF_SIZE/4;
			if(CDC_Transmit_HS((uint8_t*)(data_buf_RX_CDC+MGR_CDC.outPTR[0]),ByteToSend)==USBD_OK)
			{
				dataMGR_deQueue(&MGR_CDC,ByteToSend,0);
			}				
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 0;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 21599;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 2000000;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 3);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ARD_D7_GPIO_Pin|ARD_D8_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, WIFI_RST_Pin|WIFI_GPIO_0_Pin|PMOD_GPIO_0_Pin|USB_OTGFS_PPWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, PMOD_SPI2_MOSI_Pin|PMOD_SPI2_MISO_Pin|GPIO_PIN_1|GPIO_PIN_10 
                          |GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_CH_PD_GPIO_Port, WIFI_CH_PD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PMOD_SEL_0_GPIO_Port, PMOD_SEL_0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USB_OTG_FS_ID_Pin|SYS_LD_USER1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, PMOD_GPIO_1_Pin|ARD_D4_GPIO_Pin|USB_OTGHS_PPWR_EN_Pin|CTP_RST_Pin 
                          |LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ARD_D2_GPIO_GPIO_Port, ARD_D2_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USB_OTG_HS_ID_Pin|SYS_LD_USER2_Pin|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : ARD_D7_GPIO_Pin ARD_D8_GPIO_Pin */
  GPIO_InitStruct.Pin = ARD_D7_GPIO_Pin|ARD_D8_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PSRAM_NBL1_Pin PSRAM_NBL0_Pin LCD_PSRAM_D10_Pin LCD_PSRAM_D5_Pin 
                           LCD_PSRAM_D6_Pin LCD_PSRAM_D8_Pin LCD_PSRAM_D11_Pin LCD_PSRAM_D4_Pin 
                           LCD_PSRAM_D7_Pin LCD_PSRAM_D9_Pin LCD_PSRAM_D12_Pin */
  GPIO_InitStruct.Pin = PSRAM_NBL1_Pin|PSRAM_NBL0_Pin|LCD_PSRAM_D10_Pin|LCD_PSRAM_D5_Pin 
                          |LCD_PSRAM_D6_Pin|LCD_PSRAM_D8_Pin|LCD_PSRAM_D11_Pin|LCD_PSRAM_D4_Pin 
                          |LCD_PSRAM_D7_Pin|LCD_PSRAM_D9_Pin|LCD_PSRAM_D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_I2C1_SCL_Pin SAI2_I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_I2C1_SCL_Pin|SAI2_I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D11_TIM3_CH2_SPI1_MOSI_Pin ARD_D12_SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = ARD_D11_TIM3_CH2_SPI1_MOSI_Pin|ARD_D12_SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : WIFI_RST_Pin WIFI_GPIO_0_Pin PMOD_GPIO_0_Pin USB_OTGFS_PPWR_EN_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin|WIFI_GPIO_0_Pin|PMOD_GPIO_0_Pin|USB_OTGFS_PPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PSRAM_NE1_Pin LCD_PSRAM_D2_Pin LCD_PSRAM_NWE_Pin LCD_PSRAM_D3_Pin 
                           LCD_PSRAM_NWED4_Pin LCD_PSRAM_D1_Pin LCD_PSRAM_D0_Pin PSRAM_A17_Pin 
                           PSRAM_A16_Pin LCD_PSRAM_D15_Pin LCD_PSRAM_D14_Pin */
  GPIO_InitStruct.Pin = PSRAM_NE1_Pin|LCD_PSRAM_D2_Pin|LCD_PSRAM_NWE_Pin|LCD_PSRAM_D3_Pin 
                          |LCD_PSRAM_NWED4_Pin|LCD_PSRAM_D1_Pin|LCD_PSRAM_D0_Pin|PSRAM_A17_Pin 
                          |PSRAM_A16_Pin|LCD_PSRAM_D15_Pin|LCD_PSRAM_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_TXD_WIFI_RX_Pin */
  GPIO_InitStruct.Pin = UART_TXD_WIFI_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(UART_TXD_WIFI_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NC1_Pin */
  GPIO_InitStruct.Pin = NC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(NC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_INT_Pin */
  GPIO_InitStruct.Pin = SAI2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SAI2_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SD_B_Pin */
  GPIO_InitStruct.Pin = SAI2_SD_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SD_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_FS_A_Pin SAI2_SD_A_Pin SAI2_SCK_A_Pin SAI2_MCLK_A_Pin */
  GPIO_InitStruct.Pin = SAI2_FS_A_Pin|SAI2_SD_A_Pin|SAI2_SCK_A_Pin|SAI2_MCLK_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_NE_Pin PSRAM_A15_Pin PSRAM_A14_Pin PSRAM_A13_Pin 
                           PSRAM_A12_Pin PSRAM_A11_Pin PSRAM_A10_Pin */
  GPIO_InitStruct.Pin = LCD_NE_Pin|PSRAM_A15_Pin|PSRAM_A14_Pin|PSRAM_A13_Pin 
                          |PSRAM_A12_Pin|PSRAM_A11_Pin|PSRAM_A10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_SPI2_MOSI_Pin PMOD_SPI2_MISO_Pin PI1 PI10 
                           PI0 */
  GPIO_InitStruct.Pin = PMOD_SPI2_MOSI_Pin|PMOD_SPI2_MISO_Pin|GPIO_PIN_1|GPIO_PIN_10 
                          |GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_INT_Pin */
  GPIO_InitStruct.Pin = CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_CH_PD_Pin */
  GPIO_InitStruct.Pin = WIFI_CH_PD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_CH_PD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_RXD_WIFI_TX_Pin */
  GPIO_InitStruct.Pin = UART_RXD_WIFI_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(UART_RXD_WIFI_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_SEL_0_Pin PMOD_GPIO_1_Pin ARD_D4_GPIO_Pin USB_OTGHS_PPWR_EN_Pin 
                           CTP_RST_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = PMOD_SEL_0_Pin|PMOD_GPIO_1_Pin|ARD_D4_GPIO_Pin|USB_OTGHS_PPWR_EN_Pin 
                          |CTP_RST_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin SYS_LD_USER1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|SYS_LD_USER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PSRAM_A0_Pin PSRAM_A2_Pin PSRAM_A1_Pin PSRAM_A3_Pin 
                           PSRAM_A4_Pin PSRAM_A5_Pin PSRAM_A7_Pin PSRAM_A6_Pin 
                           PSRAM_A9_Pin PSRAM_A8_Pin */
  GPIO_InitStruct.Pin = PSRAM_A0_Pin|PSRAM_A2_Pin|PSRAM_A1_Pin|PSRAM_A3_Pin 
                          |PSRAM_A4_Pin|PSRAM_A5_Pin|PSRAM_A7_Pin|PSRAM_A6_Pin 
                          |PSRAM_A9_Pin|PSRAM_A8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_SCL_Pin */
  GPIO_InitStruct.Pin = CTP_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(CTP_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_TE_INT_Pin */
  GPIO_InitStruct.Pin = LCD_TE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_TE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VCP_RX_Pin VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin|VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_STMOD_I2C2_SCL_Pin ARD_D14_STMOD_I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = ARD_D15_STMOD_I2C2_SCL_Pin|ARD_D14_STMOD_I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_A3_ADC3_IN8_Pin */
  GPIO_InitStruct.Pin = ARD_A3_ADC3_IN8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_A3_ADC3_IN8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTGHS_OVCR_INT_Pin */
  GPIO_InitStruct.Pin = USB_OTGHS_OVCR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTGHS_OVCR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A4_Pin ARD_A5_Pin ARD_A2_Pin */
  GPIO_InitStruct.Pin = ARD_A4_Pin|ARD_A5_Pin|ARD_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STMOD_SPI2_MISOs_Pin STMOD__SPI2_MOSIs_Pin */
  GPIO_InitStruct.Pin = STMOD_SPI2_MISOs_Pin|STMOD__SPI2_MOSIs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_CLK_Pin */
  GPIO_InitStruct.Pin = QSPI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_SDA_Pin */
  GPIO_InitStruct.Pin = CTP_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(CTP_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D3_Pin */
  GPIO_InitStruct.Pin = QSPI_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D13_SPI1_SCK_Pin */
  GPIO_InitStruct.Pin = ARD_D13_SPI1_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(ARD_D13_SPI1_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D2_GPIO_Pin */
  GPIO_InitStruct.Pin = ARD_D2_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ARD_D2_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_HS_ID_Pin SYS_LD_USER2_Pin PB11 */
  GPIO_InitStruct.Pin = USB_OTG_HS_ID_Pin|SYS_LD_USER2_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_HS_VBUS_Pin USB_OTGFS_OVCR_INT_Pin */
  GPIO_InitStruct.Pin = USB_OTG_HS_VBUS_Pin|USB_OTGFS_OVCR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_RESET_Pin */
  GPIO_InitStruct.Pin = PMOD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PMOD_RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
