/**
  ******************************************************************************
  * @file    MT_CommonFunction.h
  * @author  Motus_LiuYaxin
  * @version V0.0.1
  * @date    25-September-2017
  * @brief   This file contains all the functions prototypes for the Motus firmware
  *          library.  
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MT_COMMONFUNCTION_H
#define __MT_COMMONFUNCTION_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>

#if defined (USE_STM324xG_EVAL)
  #include "stm324xg_eval.h"

#elif defined (USE_STM324x7I_EVAL) 
  #include "stm324x7i_eval.h"

#elif defined (USE_STM324x9I_EVAL) 
  #include "stm324x9i_eval.h"

#else
 #error "Please select first the Evaluation board used in your application (in Project Options)"
#endif
/** @addtogroup MT_CommonFunction_Driver
  * @{
  */

/** @addtogroup MT_CommonFunction
  * @{
  */
/* Exported variables------------------------------------------------------------*/	
extern __IO u8 Tim2_Flag;
extern CanTxMsg TxMessage;
/* Exported types ------------------------------------------------------------*/
typedef enum
{ 
  LED_OFF,
	LED_ON
}LED_STATUS_TypeDef;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void NVIC_Config(void);
void CAN_BSP_Config(void);
void TIM_Configuration(void);
//void TIM2_IRQHandler(void);
//void CAN1_RX0_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif /*__STM32F4xx_GPIO_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


