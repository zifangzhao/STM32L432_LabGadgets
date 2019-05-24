/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dataMGR.h"
#include "stm32f7xx_hal_dma.h"
#include "CE32_ioncom.h"
#include "CE32_macro.h"
/* USER CODE END Includes */
  
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern uint8_t data_buf_RX1[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX2[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX3[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX4[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_TX[BUF_SIZE];		// This records @ 20kHz x 32CH

extern dataMGR MGR_RX1;
extern dataMGR MGR_RX2;
extern dataMGR MGR_RX3;
extern dataMGR MGR_RX4;
extern dataMGR MGR_TX;

extern struct CE32_IONCOM_Handle IC_handle1;
extern struct CE32_IONCOM_Handle IC_handle2;
extern struct CE32_IONCOM_Handle IC_handle3;
extern struct CE32_IONCOM_Handle IC_handle4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart7;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
	//USART3_RX
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_usart3_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_usart3_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_usart3_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle2.RX_MGR,IC_handle2.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle2);
	if((IC_handle2.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle2.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle2.RX_MGR.dataPtr + IC_handle2.DMA_bank_in*(IC_handle2.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle2.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle2.RX_MGR.dataPtr + IC_handle2.DMA_bank_in*(IC_handle2.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
	//UART4
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_uart4_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_uart4_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_uart4_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle3.RX_MGR,IC_handle3.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle3);
	if((IC_handle3.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle3.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle3.RX_MGR.dataPtr + IC_handle3.DMA_bank_in*(IC_handle3.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle3.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle3.RX_MGR.dataPtr + IC_handle3.DMA_bank_in*(IC_handle3.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA1_Stream2_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	//USART3_RX
 	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_uart7_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_uart7_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_uart7_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle1.RX_MGR,IC_handle1.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle1);
	if((IC_handle1.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle1.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle1.RX_MGR.dataPtr + IC_handle1.DMA_bank_in*(IC_handle1.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle1.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle1.RX_MGR.dataPtr + IC_handle1.DMA_bank_in*(IC_handle1.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */
	
  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	//UsART2
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_usart2_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_usart2_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_usart2_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle4.RX_MGR,IC_handle4.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle4);
	if((IC_handle4.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle4.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle4.RX_MGR.dataPtr + IC_handle4.DMA_bank_in*(IC_handle4.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle4.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle4.RX_MGR.dataPtr + IC_handle4.DMA_bank_in*(IC_handle4.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
	uint32_t isrflags   = READ_REG(huart7.Instance->ISR); //read ISR flag
	if((isrflags&(uint32_t)(USART_ISR_RXNE))!=0)
	{
		char data=huart7.Instance->RDR&0xff; //read data
		dataMGR_enQueue_byte(&MGR_RX1,data);
	}
	//TX ISR
	if((huart7.Instance->CR1&USART_CR1_TXEIE)!=0)
	{		
		if((isrflags&(uint32_t)(USART_ISR_TXE))!=0)
		{
			if(MGR_TX.bufferUsed[0]>0)
			{
				UART7->TDR=dataMGR_deQueue_byte(&MGR_TX,0);
			}
			else
			{
				MGR_TX.logState&=~MGR_STATE_TRANSBUSY;
				CLEAR_BIT(huart7.Instance->CR1, USART_CR1_TXEIE); //clear TXE interrupt if buffer is empty
			}
		}
	}
  /* USER CODE END UART7_IRQn 0 */
  /* USER CODE BEGIN UART7_IRQn 1 */
	UART7->ICR=USART_ICR_ORECF;
  /* USER CODE END UART7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
