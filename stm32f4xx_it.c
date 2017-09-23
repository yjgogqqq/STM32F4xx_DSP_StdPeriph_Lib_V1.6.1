/**
  ******************************************************************************
  * @file    TIM/TIM_TIM9OCToggle/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "StepperMotor.h"
/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_TIM9OCToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t capture = 0;
extern __IO uint16_t CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern uint16_t SRC_Buffer_INC[20];
extern uint16_t SRC_Buffer_DEC[20];

FlagStatus acceleration = RESET;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
//	/* Change the DMA_MemoryBaseAddr to the decrement buffer */
//  DMA_MemoryTargetConfig(DMA2_Stream1, (uint32_t)SRC_Buffer_DEC,
//                            DMA_Memory_0);
//  /* Define the DMA_BufferSize */  
//  DMA_SetCurrDataCounter(DMA2_Stream1, BufferSize);
//  /* Enable DMA2_Stream5 */
//	DMA_Cmd(DMA2_Stream1,ENABLE);
//  /* Enable TIM2 DMA request */ 
//  TIM1->DIER |= TIM_DMA_Update;
//  
//  /* Disable SysTick Counter */ 
//  SysTick->CTRL &= 0xFFFFFFFE;
//  
//  /* Clear SysTick Counter */
//  SysTick->VAL = ((uint32_t)0x00000000);
//  
//  acceleration = SET;
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/

/**
  * @brief  This function handles TIM1 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
}
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  /* TIM3_CH1 toggling with frequency = 256.35 Hz */
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update );
		PulseFlag=0x01;
  }
}

void DMA2_Stream5_IRQHandler(void)
{
  /* Disable TIM9 DMA */  
  TIM1->DIER &= ~TIM_DMA_Update;
    
  /* Disable DMA Channel2 request */  
  DMA_Cmd(DMA2_Stream5,DISABLE);
  
  if(acceleration == SET)
  {       
    /* Change the DMA_MemoryBaseAddr to the decrement buffer */
		DMA_MemoryTargetConfig(DMA2_Stream5, (uint32_t)SRC_Buffer_INC,
                            DMA_Memory_0);
    /* Define the DMA_BufferSize */  
    DMA_SetCurrDataCounter(DMA2_Stream5, BufferSize);
  
    /* Enable DMA Channel2 */
    DMA_Cmd(DMA2_Stream5,ENABLE);

    /* Enable TIM2 DMA request */ 
    TIM1->DIER |= TIM_DMA_Update;
    
    acceleration = RESET; 
  }
  else
  {    
    /* SysTick end of count event each 8ms  */
    /* Setup SysTick Timer for 1 msec interrupts */
		if (SysTick_Config(SystemCoreClock/ 8000)) /* SystemFrequency is defined in ?system_stm32f10x.h? and equal to HCLK frequency */
		{
			/* Capture error */
			while (1);
		} 
	NVIC_SetPriority(SysTick_IRQn, 0x0);    
		acceleration = SET;
	}
  /* Clear DMA1 Channel2 Transfer Complete pending bit */ 
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF1);
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
