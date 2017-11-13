/**
  ******************************************************************************
  * @file    MT_CommonFunction.c
  * @author  Motus_liu
  * @version V0.0.1
  * @date    25-September-2017
  * @brief   
  *                   
  *           
  *           
  *           
  * 
@verbatim  
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================       
 [..]             
   (#)
               
   (#) 
   
   (#) 
          
   (#) 
            
   (#) 
                 
   (#) 
  
   (#) 
  
   (#) 
               
@endverbatim        
  *
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
#include "MT_CommonFunction.h"
#include <stdio.h>
/** @addtogroup Motus
  * @{
  */

/** @defgroup Motus_CommonFunction
  * @brief Motus driver modules: CAN、TIM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CanTxMsg TxMessage;
CanRxMsg RxMessage;
__IO u8 Tim2_Flag=0;
USART_TypeDef* CurrentUSARTx;
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* Private functions ---------------------------------------------------------*/

/** @defgroup Motus_CommonFunction_Private_Functions
  * @{
  */ 

/** @defgroup Tim Initialization and Configuration
 *  @brief   Initialization and Configuration
 *
@verbatim   
 ===============================================================================
                 ##### Initialization and Configuration #####
 ===============================================================================  

@endverbatim
  * @{
  */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(CurrentUSARTx, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(CurrentUSARTx, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
	
/**
  * @brief  Configures the NVIC.
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
	
	/* Configure two bits for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the TIM2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN_BSP_Config(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 
 
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);
  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate = 1 MBps (CAN clocked at 42 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
	
  CAN_InitStructure.CAN_Prescaler = 6;	 //6

  CAN_Init(CAN1, &CAN_InitStructure);

///////////////////////////////////////////////////////////////////////////////////////////////////////	
	
  /* Transmit Structure preparation */

  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

void TIM_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	/* Enable TIM2 clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);	   //TIM2_Clock 84M
  //TIM_DeInit(TIM2);
	
	/* TIM2 configuration */
  TIM_TimeBaseStructure.TIM_Period=1;		 						/* 自动重装载寄存器周期的值(计数值) */																	/* 累计 TIM_Period个频率后产生一个更新或者中断 */
  TIM_TimeBaseStructure.TIM_Prescaler= (42000 - 1);				    /* 时钟预分频数   例如：时钟频率=72MHZ/(时钟预分频+1) */
 
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 			/* 采样分频 */
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		/* 向上计数模式 */
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* Immediate load of TIM2 Precaler values */
	TIM_PrescalerConfig(TIM2, (42000 - 1), TIM_PSCReloadMode_Immediate);
	
	/* Clear TIM2 update pending flags */
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	/* Enable TIM2 Update interrupts */
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	/* TIM2 enable counters */
  TIM_Cmd(TIM2, ENABLE);											/* 开启时钟 */
}

/**
  * @brief  Turn ON/OFF the dedicated led
  * @param  Ledstatus: Led number from 0 to 3.
  * @retval None
  */
void LED_Display(uint8_t Ledstatus)
{
	
}
/**
  * @brief  Initialize COM1 interface for serial debug
  * @note   COM1 interface is defined in stm3210g_eval.h file (under Utilities\STM32_EVAL\STM324xG_EVAL)  
  * @param  None
  * @retval None
  */
void DebugComPort_Init(USART_TypeDef* USARTx)
{
  CurrentUSARTx=USARTx;
  if(USART1==USARTx)
  {
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef  USART_ClockInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    /* Configure USARTx_Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure USARTx_Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate =115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE); 
  }
  else if(USART3==USARTx)
  {
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef  USART_ClockInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
    /* Configure USARTx_Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* Configure USARTx_Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate =115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE); 
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
u8 Uart_PutChar(u8 ch)
{
    
	USART_SendData(USART1, (u8) ch);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
	return ch;

}
void Uart_PutString(u8 buf[],u16 len)
{
    u16 i;
    for( i=0;i<len;i++)
    {
        Uart_PutChar(*buf++);
	}

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

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
