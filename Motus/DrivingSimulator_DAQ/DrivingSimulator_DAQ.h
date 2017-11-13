/**
  ******************************************************************************
  * @file    DrivingSimulator_DAQ.c
  * @author  Motus_liu
  * @version V0.0.1
  * @date    25-September-2017
  * @brief
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
#ifndef __DRIVINGSIMULATOR_DAQ_H
#define __DRIVINGSIMULATOR_DAQ_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "MT_CommonFunction.h"
#if defined (USE_STM324xG_EVAL)
  #include "stm324xg_eval.h"

#elif defined (USE_STM324x7I_EVAL) 
  #include "stm324x7i_eval.h"

#elif defined (USE_STM324x9I_EVAL) 
  #include "stm324x9i_eval.h"

#else
 #error "Please select first the Evaluation board used in your application (in Project Options)"
#endif

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup GPIO
  * @{
  */ 
	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
//AD			模拟量采集
#define		WIPER_INT_TIME_PIN														GPIOC, GPIO_Pin_0
#define		PARKING_BRAKE_PIN															GPIOC, GPIO_Pin_1
#define		CLUTCH_PIN																		GPIOC, GPIO_Pin_2
#define		BRAKE_PIN																			GPIOC, GPIO_Pin_3                                                       
#define		ACCELERATOR_PEDAL_PIN													GPIOA, GPIO_Pin_0
#define		SEAT_ADJUST_PIN																GPIOA, GPIO_Pin_1                                                        
#define		INDOOR_REARVIEW_X_AXIS_PIN										GPIOA, GPIO_Pin_2
#define		INDOOR_REARVIEW_Y_AXIS_PIN										GPIOA, GPIO_Pin_3                                                      
#define		LEFT_REARVIEW_X_AXIS_PIN											GPIOA, GPIO_Pin_4
#define		LEFT_REARVIEW_Y_AXIS_PIN											GPIOA, GPIO_Pin_5                                                         
#define		RIGHT_REARVIEW_X_AXIS_PIN											GPIOA, GPIO_Pin_6
#define		RIGHT_REARVIEW_Y_AXIS_PIN											GPIOA, GPIO_Pin_7
#define		AD_RESERVE0_PIN																GPIOC, GPIO_Pin_4
#define		AD_RESERVE1_PIN																GPIOC, GPIO_Pin_5                                                          
#define		AD_RESERVE2_PIN																GPIOB, GPIO_Pin_0
#define		AD_RESERVE3_PIN																GPIOB, GPIO_Pin_1

#define		WIPER_INT_TIME														11
#define		PARKING_BRAKE															10
#define		CLUTCH																		13 
#define		BRAKE																			12                                                      
#define		ACCELERATOR_PEDAL													1
#define		SEAT_ADJUST																0                                                        
#define		INDOOR_REARVIEW_X_AXIS										3
#define		INDOOR_REARVIEW_Y_AXIS										2                                                      
#define		LEFT_REARVIEW_X_AXIS											5
#define		LEFT_REARVIEW_Y_AXIS											4                                                         
#define		RIGHT_REARVIEW_X_AXIS											7
#define		RIGHT_REARVIEW_Y_AXIS											6
#define		AD_RESERVE0																15
#define		AD_RESERVE1																14                                                          
#define		AD_RESERVE2																9
#define		AD_RESERVE3																8


//D_IO		输入输出引脚
#define		SMALL_LIGHT_SW_PIN														GPIOE, GPIO_Pin_7
#define		LARGE_LIGHT_SW_PIN														GPIOE, GPIO_Pin_8                                                          
#define		AUTO_LIGHT_SW_PIN															GPIOE, GPIO_Pin_9
#define		FAR_LIGHT_SW_PIN															GPIOE, GPIO_Pin_10                                                         
#define		SHORT_LIGHT_SW_PIN														GPIOE, GPIO_Pin_11
#define		LEFT_TURN_LIGHT_SW_PIN												GPIOE, GPIO_Pin_12                                                      
#define		RIGHT_TURN_LIGHT_SW_PIN												GPIOE, GPIO_Pin_13
#define		FOG_LIGHT_SW_PIN															GPIOE, GPIO_Pin_14                                                     

#define		HORN_SW_PIN																		GPIOE, GPIO_Pin_15
#define		KEY_ACC_STATE_PIN															GPIOB, GPIO_Pin_10                                                      
#define		KEY_ON_STATE_PIN															GPIOB, GPIO_Pin_11
#define		KEY_START_STATE_PIN														GPIOB, GPIO_Pin_12
#define		WIPER_INT_STATE_PIN														GPIOB, GPIO_Pin_13
#define		WIPER_LO_STATE_PIN														GPIOB, GPIO_Pin_14
#define		WIPER_HI_STATE_PIN														GPIOB, GPIO_Pin_15
#define		SPRAYER_SW_PIN																GPIOD, GPIO_Pin_10

#define		BACK_WIPER_ONCE_STATE_PIN											GPIOD, GPIO_Pin_11
#define		BACK_WIPER_OFF_STATE_PIN											GPIOD, GPIO_Pin_12
#define		BACK_WIPER_INT_STATE_PIN											GPIOD, GPIO_Pin_13
#define		BACK_WIPER_ON_STATE_PIN												GPIOD, GPIO_Pin_14
#define		BACK_WIPER_SPRAYER_STATE_PIN									GPIOD, GPIO_Pin_15
#define		SECOND_GEAR_PIN																GPIOC, GPIO_Pin_6
#define		FOURTH_GEAR_PIN																GPIOC, GPIO_Pin_7
#define		REVERSE_GEAR_PIN																GPIOC, GPIO_Pin_8

#define		FIFTH_GEAR_PIN																GPIOC, GPIO_Pin_9
#define		THIRD_GEAR_PIN																GPIOA, GPIO_Pin_8
#define		FIRST_GEAR_PIN															GPIOA, GPIO_Pin_11
#define		SAFETY_BELT_PIN																GPIOA, GPIO_Pin_12
#define		HAZARD_WARNING_LAMP_PIN												GPIOC, GPIO_Pin_10
#define		RESERVE0_PIN																	GPIOC, GPIO_Pin_11
#define		RESERVE1_PIN																	GPIOC, GPIO_Pin_12
#define		RESERVE2_PIN																	GPIOD, GPIO_Pin_2
#define		RESERVE3_PIN																	GPIOD, GPIO_Pin_3                                                         
#define		RESERVE4_PIN																	GPIOD, GPIO_Pin_4
#define		RESERVE5_PIN																	GPIOD, GPIO_Pin_7                                                       
#define		RESERVE6_PIN																	GPIOB, GPIO_Pin_5
#define		RESERVE7_PIN																	GPIOB, GPIO_Pin_6                                                      
#define		RESERVE8_PIN																	GPIOB, GPIO_Pin_7
#define		RESERVE9_PIN																	GPIOB, GPIO_Pin_8                                                       
#define		RESERVE10_PIN																	GPIOB, GPIO_Pin_9
#define		RESERVE11_PIN																	GPIOE, GPIO_Pin_0
#define		RESERVE12_PIN																	GPIOE, GPIO_Pin_1
#define		RESERVE13_PIN																	GPIOE, GPIO_Pin_2
#define		RESERVE14_PIN																	GPIOE, GPIO_Pin_3
#define		RESERVE15_PIN																	GPIOE, GPIO_Pin_4
#define		RESERVE16_PIN																	GPIOE, GPIO_Pin_5
#define		RESERVE17_PIN																	GPIOE, GPIO_Pin_6
#define		RESERVE18_PIN																	GPIOC, GPIO_Pin_13

#define		SMALL_LIGHT_SW_STATUS														GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7)
#define		LARGE_LIGHT_SW_STATUS														GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8)                                                       
#define		AUTO_LIGHT_SW_STATUS															GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9)
#define		FAR_LIGHT_SW_STATUS															GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10)                                                        
#define		SHORT_LIGHT_SW_STATUS														GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)
#define		LEFT_TURN_LIGHT_SW_STATUS												GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)                                                   
#define		RIGHT_TURN_LIGHT_SW_STATUS												GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13)
#define		FOG_LIGHT_SW_STATUS															GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14)                                                  

#define		HORN_SW_STATUS																		GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15)
#define		KEY_ACC_STATUS															GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)                                                      
#define		KEY_ON_STATUS															GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)
#define		KEY_START_STATUS														GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)
#define		WIPER_INT_STATUS														GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)
#define		WIPER_LO_STATUS														GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)
#define		WIPER_HI_STATUS														GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)
#define		SPRAYER_SW_STATUS																GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)

#define		BACK_WIPER_ONCE_STATUS											GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11)
#define		BACK_WIPER_OFF_STATUS											GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12)
#define		BACK_WIPER_INT_STATUS											GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)
#define		BACK_WIPER_ON_STATUS												GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14)
#define		BACK_WIPER_SPRAYER_STATUS									GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15)
#define		FIRST_GEAR_STATUS																GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)
#define		SECOND_GEAR_STATUS																GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)
#define		THIRD_GEAR_STATUS																GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)

#define		FOURTH_GEAR_STATUS																GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)
#define		FIFTH_GEAR_STATUS																GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)
#define		REVERSE_GEAR_STATUS															GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)
#define		SAFETY_BELT_STATUS																GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)
#define		HAZARD_WARNING_LAMP_STATUS												GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10)
#define		RESERVE0_STATUS																	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11)
#define		RESERVE1_STATUS																	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)
#define		RESERVE2_STATUS																	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)

#define		RESERVE3_STATUS																	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)                                                        
#define		RESERVE4_STATUS																	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)
#define		RESERVE5_STATUS																	GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7)                                                      
#define		RESERVE6_STATUS																	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define		RESERVE7_STATUS																	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)                                                      
#define		RESERVE8_STATUS																	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)
#define		RESERVE9_STATUS																	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)                                                      
#define		RESERVE10_STATUS																GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)
#define		RESERVE11_STATUS																GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)
#define		RESERVE12_STATUS																GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)
#define		RESERVE13_STATUS																GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2)
#define		RESERVE14_STATUS																GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)
#define		RESERVE15_STATUS																GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)
#define		RESERVE16_STATUS																GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)
#define		RESERVE17_STATUS																GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)
#define		RESERVE18_STATUS																GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)
//LED			闪烁灯对应的引脚
#define		DAQ_LED1_PIN																	GPIOC, GPIO_Pin_14
#define		DAQ_LED2_PIN																	GPIOC, GPIO_Pin_15

#define		DAQ_LED1_STATUS_SET(X)												GPIO_WriteBit(GPIOC, GPIO_Pin_14, X)
#define		DAQ_LED2_STATUS_SET(X)												GPIO_WriteBit(GPIOC, GPIO_Pin_15, X)

#define		DAQ_LED1_TOGGLE												        GPIO_ToggleBits(GPIOC, GPIO_Pin_14)
#define		DAQ_LED2_TOGGLE												        GPIO_ToggleBits(GPIOC, GPIO_Pin_15)
//USART3	串口3对应的引脚
#define		U3_RXD																				GPIOC, GPIO_Pin_14
#define		U3_TXD																				GPIOC, GPIO_Pin_15

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int DrivingSimulator_DAQ_Main(void);
#ifdef __cplusplus
}
#endif

#endif /* __DRIVINGSIMULATOR_DAQ_H */


/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
