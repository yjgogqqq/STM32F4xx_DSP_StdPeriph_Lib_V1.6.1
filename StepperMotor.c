/**
  ******************************************************************************
  * @file FullHalfStepMode/src/StepperMotor.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  This file contains the stepper motor functions body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Private define ------------------------------------------------------------*/



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "StepperMotor.h"


/* Private typedef -------------------------------------------------------------*/

/* Private define --------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------*/ 
int				nVerticalSpeedStepperCurPos							=0;
int				nAttitudeStepperCurPos									=0;
int				nAirSpeedStepperCurPos									=0;
int				nEngineTachSteppperCurPos								=0;
int				nRotorTachSteppperCurPos								=0;
int				nAltimeterSteppperCurPos								=0;
int				nCompassSteppperCurPos									=0;
int				nManifoldPressureSteppperCurPos					=0;
int				nAmmeterSteppperCurPos									=0;
int				nOilPressureSteppperCurPos							=0;
int				nAuxFuelQuantitySteppperCurPos					=0;
int				nOilTemperatureSteppperCurPos						=0;
int				nMainFuelQuantitySteppperCurPos					=0;
int				nCylinderHeadTemperatureSteppperCurPos	=0;
int				nCarburetorTemperatureSteppperCurPos		=0;
int				nAirTemperatureSteppperCurPos						=0;
unsigned int	uiLightsFlag												=0;


int				nVerticalSpeedStepperDstPos							=355;
int				nAttitudeStepperDstPos									=-420;
int				nAirSpeedStepperDstPos									=0;
int				nEngineTachSteppperDstPos								=-30;
int				nRotorTachSteppperDstPos								=60;
int				nAltimeterSteppperDstPos								=560;//0x50;
int				nCompassSteppperDstPos									=100;//0x50;
int				nManifoldPressureSteppperDstPos					=0;//0x50;
int				nAmmeterSteppperDstPos									=0;//0x50;
int				nOilPressureSteppperDstPos							=0;//0x50;
int				nAuxFuelQuantitySteppperDstPos					=0;//0x50;
int				nOilTemperatureSteppperDstPos						=0;
int				nMainFuelQuantitySteppperDstPos					=0;
int				nCylinderHeadTemperatureSteppperDstPos	=0;
int				nCarburetorTemperatureSteppperDstPos		=0;//6200һȦ
int				nAirTemperatureSteppperDstPos						=0;

uint8_t		PulseFlag																=0x00;
u8 CanRevFlag=0;
uint16_t SRC_Buffer_DEC[20] ={15000,15000,20000,20000,25000,25000,30000,30000,35000,35000,\
                         40000,40000,45000,45000,50000,50000,55000,55000,60000,60000};

uint16_t SRC_Buffer_INC[20] ={60000,60000,55000,55000,50000,50000,45000,45000,40000,40000,\
                         35000,35000,30000,30000,25000,25000,20000,20000,15000,15000};
                                          
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Configures the driver control pin 
  * @param  None
  * @retval : None
  */

void Stepper_PinControlConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOG Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF| RCC_AHB1Periph_GPIOC| RCC_AHB1Periph_GPIOE| RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_GPIOG, ENABLE);
	
   /* Configure PC.04, PC.05, PC.06, PC.07, PC.08, PC.09 as output push pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 \
																| GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
                                
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	
  GPIO_Init(GPIOF, &GPIO_InitStructure);  
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 \
																| GPIO_Pin_5 |GPIO_Pin_6;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 \
																| GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11|GPIO_Pin_12 | GPIO_Pin_13|GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/**
  * @brief  Starts or Stops the stepper motor
  * @param NewState: This parameter can be ENABLE or DISABLE.
  * @retval : None
  */
void Stepper_Start(FunctionalState NewState)
{   
  if(NewState == ENABLE)
  {
     /* Set the C.09 pin */
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
  }
  else
  {   
    /* Reset the C.09 pin */
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  }
}



/**
  * @brief  Disables the initialization of the driver to its default reset value
  * @param  None
  * @retval : None
  */
void Stepper_ResetDisable(void)
{   
  /* Set the C.05pin */
  GPIO_SetBits(GPIOC, GPIO_Pin_5);
}



/**
  * @brief  Selects the direction of the rotation 
  * @param Stepper_RotationDirection: Specifies the rotation direction
  *   This parameter can be one of the following values:
  * @arg Stepper_RotationDirection_CW : sets clockwise direction
  * @arg Stepper_RotationDirection_CCW: sets counterclockwise direction
  *   
  * @retval : None
  */
void Stepper_SetRotationDirection(uint16_t Stepper_RotationDirection)
{
   if(Stepper_RotationDirection == Stepper_RotationDirection_CW )
  {
    /* Set the A.12 pin */
    GPIO_SetBits(GPIOA, GPIO_Pin_12);
  }
  else
  {   
    /* Reset the A.12 pin */
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
  }
}
void StepperHandler(eInstrumentSort teInstrument,int *tipCurPos,int *tipDstPos)
{
		if((*tipCurPos-*tipDstPos)>0)
		{
			StepperConfig(teInstrument,DirectionCCW,ENABLE);
			(*tipCurPos)--;
			//printf("*tipCurPos:%d\r\n",*tipCurPos);	
		}
		else if((*tipCurPos-*tipDstPos)<0)
		{
			StepperConfig(teInstrument,DirectionCW,ENABLE);
			(*tipCurPos)++;
			//printf("*tipCurPos:%d\r\n",*tipCurPos);	
		}
		else
		{
			StepperConfig(teInstrument,DirectionCCW,DISABLE);
		}
}
/**
  * @brief  Init GPIO
  * @param
  * @arg 
  * @arg 
  *   
  * @retval : None
  */
void StepperConfig(eInstrumentSort eInstrument,eStepperRotationDirection eDirection,FunctionalState NewState)
{
	switch((unsigned int)eInstrument)
	{
		case VERTICAL_SPEED_INDICATOR:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(VERTICAL_SPEED_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(VERTICAL_SPEED_STEPPER_DIR_PIN);
			}
			if(ENABLE==NewState)
			{
				GPIO_ResetBits(VERTICAL_SPEED_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_SetBits(VERTICAL_SPEED_STEPPER_ENABLE_PIN);
				
			}
			break;
		case ATTITUDE_INDICATOR:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(ATTITUDE_INDICATOR_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(ATTITUDE_INDICATOR_STEPPER_DIR_PIN);
			}
			if(ENABLE==NewState)
			{
				GPIO_ResetBits(ATTITUDE_INDICATOR_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_SetBits(ATTITUDE_INDICATOR_STEPPER_ENABLE_PIN);
				
			}
			break;
		case AIRSPEED_INDICATOR:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(AIRSPEED_INDICATOR_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(AIRSPEED_INDICATOR_STEPPER_DIR_PIN);
			}
			if(ENABLE==NewState)
			{
				GPIO_ResetBits(AIRSPEED_INDICATOR_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_SetBits(AIRSPEED_INDICATOR_STEPPER_ENABLE_PIN);
				
			}
			break;
		case ENGINE_TACH:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(ENGINE_TACH_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(ENGINE_TACH_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(ENGINE_TACH_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(ENGINE_TACH_STEPPER_ENABLE_PIN);
			}
			break;
		case ROTOR_TACH:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(ROTOR_TACH_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(ROTOR_TACH_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(ROTOR_TACH_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(ROTOR_TACH_STEPPER_ENABLE_PIN);
			}
			break;
		case ALTIMETER_INDICATOR:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(ALTIMETER_INDICATOR_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(ALTIMETER_INDICATOR_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(ALTIMETER_INDICATOR_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(ALTIMETER_INDICATOR_STEPPER_ENABLE_PIN);
			}
			break;
		case COMPASS_INDICATOR:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(COMPASS_INDICATOR_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(COMPASS_INDICATOR_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(COMPASS_INDICATOR_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(COMPASS_INDICATOR_STEPPER_ENABLE_PIN);
			}
			break;
		case MANIFOLD_PRESSURE_INDICATOR:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(MANIFOLD_PRESSURE_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(MANIFOLD_PRESSURE_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(MANIFOLD_PRESSURE_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(MANIFOLD_PRESSURE_STEPPER_ENABLE_PIN);
			}
			break;
		case AMMETER_INDICATOR:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(AMMETER_INDICATOR_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(AMMETER_INDICATOR_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(AMMETER_INDICATOR_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(AMMETER_INDICATOR_STEPPER_ENABLE_PIN);
			}
			break;
		case OIL_PRESSURE:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(OIL_PRESSURE_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(OIL_PRESSURE_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(OIL_PRESSURE_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(OIL_PRESSURE_STEPPER_ENABLE_PIN);
			}
			break;
		case AUX_Fuel_Quantity:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(AUX_Fuel_Quantity_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(AUX_Fuel_Quantity_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(AUX_Fuel_Quantity_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(AUX_Fuel_Quantity_STEPPER_ENABLE_PIN);
			}
			break;
		case OIL_TEMPERATURE:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(OIL_TEMPERATURE_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(OIL_TEMPERATURE_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(OIL_TEMPERATURE_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(OIL_TEMPERATURE_STEPPER_ENABLE_PIN);
			}
			break;
		case MAIN_FUEL_QUANTITY:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(MAIN_FUEL_QUANTITY_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(MAIN_FUEL_QUANTITY_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(MAIN_FUEL_QUANTITY_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(MAIN_FUEL_QUANTITY_STEPPER_ENABLE_PIN);
			}
			break;
		case CYLINDER_HEAD_TEMPERATURE:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(CYLINDER_HEAD_TEMPERATURE_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(CYLINDER_HEAD_TEMPERATURE_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(CYLINDER_HEAD_TEMPERATURE_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(CYLINDER_HEAD_TEMPERATURE_STEPPER_ENABLE_PIN);
			}
			break;
		case CARBURETOR_TEMPERATURE:
			if(DirectionCW==eDirection)
			{
				GPIO_SetBits(CARBURETOR_TEMPERATURE_STEPPER_DIR_PIN);
			}
			else
			{
				GPIO_ResetBits(CARBURETOR_TEMPERATURE_STEPPER_DIR_PIN);
			}
			if(ENABLE!=NewState)
			{
				GPIO_SetBits(CARBURETOR_TEMPERATURE_STEPPER_ENABLE_PIN);
			}
			else
			{
				GPIO_ResetBits(CARBURETOR_TEMPERATURE_STEPPER_ENABLE_PIN);
			}
			break;
	}
}
/**
  * @brief  Selects the direction of the rotation 
  * @param Stepper_RotationDirection: Specifies the rotation direction
  *   This parameter can be one of the following values:
  * @arg Stepper_RotationDirection_CW : sets clockwise direction
  * @arg Stepper_RotationDirection_CCW: sets counterclockwise direction
  *   
  * @retval : None
  */
void VerticalSpeedStepper_SetRotationDirection(uint16_t Stepper_RotationDirection)
{
   if(Stepper_RotationDirection == Stepper_RotationDirection_CW )
  {
    /* Set the A.12 pin */
    
  }
  else
  {   
    /* Reset the A.12 pin */
    
  }
}

/**
  * @brief  Selects the step mode  
  * @param Stepper_Mode: Specifies the Step Mode
  *   This parameter can be one of the following values:
  * @param Stepper_Full : Sets FULL STEP Mode
  * @param Stepper_Half : Sets HALF STEP Mode
  *   
  * @retval : None
  */
void Stepper_SelectMode(uint16_t Stepper_Mode)
{
  if(Stepper_Mode ==  Stepper_Full)
  {
     /* Reset the C.07 pin */
   GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  }
  else
  {   
    /* Set the C.07 pin */
   GPIO_SetBits(GPIOC, GPIO_Pin_7);
  }
}



/**
  * @brief  Selects the decay mode  
  * @param StepperControlMode: Specifies the Decay Mode
  *   This parameter can be one of the following values:
  * @param Stepper_ControlFast : Sets FAST DECAY Mode
  * @param Stepper_ControlSlow : Sets SLOW DECAY Mode
  *   
  * @retval : None
  */
void Stepper_SetControlMode(uint16_t Stepper_ControlMode)
{
   if(Stepper_ControlMode ==  Stepper_ControlFast)
  {
    /* Reset the C.08 pin */
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  }
  else
  {   
   /* Set the C.08 pin */
   GPIO_SetBits(GPIOC, GPIO_Pin_8);
  }

}


/**
  * @brief  Activates or Desactivates the driver
  * @param NewState: This parameter can be ENABLE or DISABLE.
  * @retval : None
  */
void Stepper_Cmd(FunctionalState NewState)
{   
    if(NewState == ENABLE)
  {
    /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
    /* GPIOC clock enable */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    /* Enable DMA1 clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  }
  else
  {   
    /* TIM2 clock disable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
     /* GPIOA clock disable */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
    /* GPIOC clock disable */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
    /* Disable DMA1 clock */
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
   
  }
}



/**
  * @brief  Configures the peripherals used to control the stepper motor 
  * @param  None
  * @retval : None
  */

void StepperInit(void)
{
	Stepper_PinControlConfig();
}

//void Stepper_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	DMA_InitTypeDef DMA_InitStructure;
//   /* TIM1 clock enable */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

//  /* GPIOA clock enable */
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

//  /* DMA2 clock enable */
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//	
//	/* -------------------------------------------------------------------
//  DMA1 configuration 
//  ---------------------------------------------------------------------- */
//  
//	/* DeInitialize the DMA2 Stream5 */
//  DMA_DeInit(DMA2_Stream5);

//	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE)
//  {
//  }
//	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM1->ARR;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SRC_Buffer_INC;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//  DMA_InitStructure.DMA_BufferSize = BufferSize;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
////	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
////  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
////  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
////  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  DMA_Init(DMA2_Stream5, &DMA_InitStructure);
//  
//  /* Enable DMA1 Channel2 Transfer Complete interrupt */
//  DMA_ITConfig(DMA2_Stream5, DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_FE, ENABLE);
//  
//  /* Enable DMA1 Channel2 */
//  DMA_Cmd(DMA2_Stream5, ENABLE);
//  while ((DMA_GetCmdStatus(DMA2_Stream5) != ENABLE))
//  {
//  }
//	/* GPIOA Configuration: PA8(TIM1 CH1) as alternate function push-pull */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//	/* Connect TIM pins to AF1 */
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
//	
//	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//		/* Enable the TIM1 global Interrupt */
////		NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
////		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
////		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
////		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////		NVIC_Init(&NVIC_InitStructure);
//	
//	/* Enable the TIM1 global Interrupt */
//		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//  /* ---------------------------------------------------------------
//  TIM2 Configuration: Output Compare Toggle Mode:
//  --------------------------------------------------------------- */
//   /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = 60000;
//  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 24000000) - 1;	//2;
//  //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	
//	/* Set the Repetition counter value */
//  //TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
//	
//  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//	
//		
///******** Configure the Internal Clock source *********************************/
// /* Disable slave mode to clock the prescaler directly with the internal clock
// if the TIM_SMCR is in the reset value, we can delete the following instruction*/
//	//TIM1->SMCR =  RESET; 
//	/****************** Configure the One Pulse Mode ******************************/

// /* Select the OPM Mode */	
// //TIM1->CR1 |= TIM_CR1_OPM;  
//	
// /* Configure the PWM mode:
//       + Channel : Channel 1
//       + PWM Mode : mode 2
//       + Polarity : High
// */
// /* Select the Channel 1 Output Compare and the Mode */ 
// //TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_OC1M;    
// //TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_CC1S;
// //TIM1->CCMR1 |=  ((uint32_t)TIM_CCMR1_OC1M);  
//	
// /* Set the Output Compare Polarity to High */
// //TIM1->CCER &= (uint16_t)~TIM_CCER_CC1P;     
// //TIM1->CCER |= ((uint32_t)0x0000); 
//           
// /* Enable the Compare output channel 1*/
// //TIM1->CCER = TIM_CCER_CC1E;
// 	
// /*Enable the TIM main Output*/
// //TIM1->BDTR |= TIM_BDTR_MOE; 

// /*Enable the TIM peripheral*/
// //TIM1->CR1 |= TIM_CR1_CEN; 
//	
//  /* Output Compare Toggle Mode configuration: Channel1 */
//  /* Output Compare Toggle Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = 0;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//	
//	/* TIM1 DMAR Base register and DMA Burst Length Config */
//  TIM_DMAConfig(TIM1, TIM_DMABase_ARR, TIM_DMABurstLength_1Transfer);
//	/* TIM1 DMA Update enable */
//  TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
//	//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
//  //TIM_ARRPreloadConfig(TIM1, ENABLE);
//  
//  /* TIM enable counter */
//  TIM_Cmd(TIM1, ENABLE);
//  /* TIM1 PWM Outputs Enable */
//  TIM_CtrlPWMOutputs(TIM1, ENABLE);

//  /* Enable DMA2 Stream5  */
//  DMA_Cmd(DMA2_Stream5, ENABLE);

//  /* Wait until DMA2 Stream5 end of Transfer */
//  while (!DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5))
//  {
//  }
//  
//	/* Enable the DMA Stream IRQ Channel */
//  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);     
//  /* Enable TIM1 DMA update request */
//  TIM_DMACmd(TIM1,TIM_DMA_Update|TIM_DMA_Trigger|TIM_DMA_COM, ENABLE);

//}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

