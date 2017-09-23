/**
  ******************************************************************************
  * @file    TIM/TIM_TIM9OCToggle/main.c 
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   Main program body
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
#include "stm32f4xx.h"
#include "StepperMotor.h"
//#include "can.h"
/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_TIM9OCToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ROTOR_CLUTCH_LIGHT(X)											(X)?GPIO_ResetBits(ROTOR_CLUTCH_LIGHT_PIN):GPIO_SetBits(ROTOR_CLUTCH_LIGHT_PIN)
#define	MR_TEMP_LIGHT(X)													(X)?GPIO_ResetBits(MR_TEMP_LIGHT_PIN):GPIO_SetBits(MR_TEMP_LIGHT_PIN)
#define	MR_CHIP_LIGHT(X)													(X)?GPIO_ResetBits(MR_CHIP_LIGHT_PIN):GPIO_SetBits(MR_CHIP_LIGHT_PIN)
#define	ROTOR_BRAKE_LIGHT(X)											(X)?GPIO_ResetBits(ROTOR_BRAKE_LIGHT_PIN):GPIO_SetBits(ROTOR_BRAKE_LIGHT_PIN)
#define	STARTER_LIGHT(X)													(X)?GPIO_ResetBits(STARTER_LIGHT_PIN):GPIO_SetBits(STARTER_LIGHT_PIN)
#define	TR_CHIP_LIGHT(X)													(X)?GPIO_ResetBits(TR_CHIP_LIGHT_PIN):GPIO_SetBits(TR_CHIP_LIGHT_PIN)
#define	LOW_FUEL_LIGHT(X)													(X)?GPIO_ResetBits(LOW_FUEL_LIGHT_PIN):GPIO_SetBits(LOW_FUEL_LIGHT_PIN)
#define	LOW_ROTOR_RPM_LIGHT(X)										(X)?GPIO_ResetBits(LOW_ROTOR_RPM_LIGHT_PIN):GPIO_SetBits(LOW_ROTOR_RPM_LIGHT_PIN)
#define	ALTERNATOR_LIGHT(X)												(X)?GPIO_ResetBits(ALTERNATOR_LIGHT_PIN):GPIO_SetBits(ALTERNATOR_LIGHT_PIN)
#define	LOW_OIL_PRESSURE_LIGHT(X)									(X)?GPIO_ResetBits(LOW_OIL_PRESSURE_LIGHT_PIN):GPIO_SetBits(LOW_OIL_PRESSURE_LIGHT_PIN)
#define	ROTOR_GOVERNER_LIGHT(X)										(X)?GPIO_ResetBits(ROTOR_GOVERNER_LIGHT_PIN):GPIO_SetBits(ROTOR_GOVERNER_LIGHT_PIN)
#define	ENG_FIRE_LIGHT(X)													(X)?GPIO_ResetBits(ENG_FIRE_LIGHT_PIN):GPIO_SetBits(ENG_FIRE_LIGHT_PIN)
#define	AUX_FUEL_PUMP_LIGHT(X)										(X)?GPIO_ResetBits(AUX_FUEL_PUMP_LIGHT_PIN):GPIO_SetBits(AUX_FUEL_PUMP_LIGHT_PIN)
#define	FUEL_FILTER_LIGHT(X)											(X)?GPIO_ResetBits(FUEL_FILTER_LIGHT_PIN):GPIO_SetBits(FUEL_FILTER_LIGHT_PIN)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t PrescalerValue = 0;
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

uint8_t SensorFlag1=0;
uint8_t SensorFlag2=0;
/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
void TIM_Config(void);
static void Delay(__IO uint32_t nTime);
void IndicatorLightInit(void);
void InstrumentPointerToZero();
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f40_41xxx.s/startup_stm32f427_437xx.s/startup_stm32f429_439xx.s)
       before to branch to application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(SystemCoreClock / 1000);
	uart_init(115200);		//初始化串口波特率为115200
	
	StepperInit();
	TIM_Config();
	IndicatorLightInit();
	
	InstrumentPointerToZero();
	
	CAN1_BSP_Init();
	//while(1);
  while (1)
  {
		if(((355-1)==nVerticalSpeedStepperDstPos)&&((-420-1)==nAttitudeStepperDstPos)&&((0-1)==nAirSpeedStepperDstPos) \
			&&((0-1)==nEngineTachSteppperDstPos)&&((0-1)==nRotorTachSteppperDstPos)&&(( 560-1)==nAltimeterSteppperDstPos) \
		&&((0-1)==nCompassSteppperDstPos)&&((0-1)==nManifoldPressureSteppperDstPos)&&((0-1)==nAmmeterSteppperDstPos))
		{
			InstrumentPointerToZero();
		}
		if(0x01==PulseFlag)
		{
			PulseFlag=0x00;
			StepperHandler(VERTICAL_SPEED_INDICATOR,&nVerticalSpeedStepperCurPos,&nVerticalSpeedStepperDstPos);
			StepperHandler(ATTITUDE_INDICATOR,&nAttitudeStepperCurPos,&nAttitudeStepperDstPos);
			StepperHandler(AIRSPEED_INDICATOR,&nAirSpeedStepperCurPos,&nAirSpeedStepperDstPos);
			
			StepperHandler(ENGINE_TACH,&nEngineTachSteppperCurPos,&nEngineTachSteppperDstPos);
			StepperHandler(ROTOR_TACH,&nRotorTachSteppperCurPos,&nRotorTachSteppperDstPos);
			if(0==GPIO_ReadInputDataBit(ROLL_SENSOR_INPUT))
			{
				nAltimeterSteppperCurPos=0;
			}
			StepperHandler(ALTIMETER_INDICATOR,&nAltimeterSteppperCurPos,&nAltimeterSteppperDstPos);
			
			if(1==GPIO_ReadInputDataBit(COMPASS_SENSOR_INPUT))
			{
				//nCompassSteppperCurPos=0;
			}
			StepperHandler(COMPASS_INDICATOR,&nCompassSteppperCurPos,&nCompassSteppperDstPos);
			
			StepperHandler(MANIFOLD_PRESSURE_INDICATOR,&nManifoldPressureSteppperCurPos,&nManifoldPressureSteppperDstPos);
			StepperHandler(AMMETER_INDICATOR,&nAmmeterSteppperCurPos,&nAmmeterSteppperDstPos);
			StepperHandler(OIL_PRESSURE,&nOilTemperatureSteppperCurPos,&nOilTemperatureSteppperDstPos);
			StepperHandler(AUX_Fuel_Quantity,&nAuxFuelQuantitySteppperCurPos,&nAuxFuelQuantitySteppperDstPos);
			StepperHandler(OIL_TEMPERATURE,&nOilTemperatureSteppperCurPos,&nOilTemperatureSteppperDstPos);
			StepperHandler(MAIN_FUEL_QUANTITY,&nMainFuelQuantitySteppperCurPos,&nMainFuelQuantitySteppperDstPos);
			StepperHandler(CYLINDER_HEAD_TEMPERATURE,&nCylinderHeadTemperatureSteppperCurPos,&nCylinderHeadTemperatureSteppperCurPos);
			if((0==GPIO_ReadInputDataBit(ALTIMETER_SENSOR_INPUT))&&(3000<nCarburetorTemperatureSteppperCurPos))
			{
				nCarburetorTemperatureSteppperCurPos=6200;
			}else if((0==GPIO_ReadInputDataBit(ALTIMETER_SENSOR_INPUT))&&(3000>=nCarburetorTemperatureSteppperCurPos))
			{
				nCarburetorTemperatureSteppperCurPos=0;
			}
			
			StepperHandler(CARBURETOR_TEMPERATURE,&nCarburetorTemperatureSteppperCurPos,&nCarburetorTemperatureSteppperDstPos);
			
			
			ROTOR_CLUTCH_LIGHT((uiLightsFlag>>0) & 0x0001);
			MR_TEMP_LIGHT((uiLightsFlag>>1) & 0x0001);
			MR_CHIP_LIGHT((uiLightsFlag>>2) & 0x0001);
			ROTOR_BRAKE_LIGHT((uiLightsFlag>>3) & 0x0001);
			STARTER_LIGHT((uiLightsFlag>>4) & 0x0001);
			TR_CHIP_LIGHT((uiLightsFlag>>5) & 0x0001);
			LOW_FUEL_LIGHT((uiLightsFlag>>6) & 0x0001);
			LOW_ROTOR_RPM_LIGHT((uiLightsFlag>>7) & 0x0001);
			ALTERNATOR_LIGHT((uiLightsFlag>>8) & 0x0001);
			LOW_OIL_PRESSURE_LIGHT((uiLightsFlag>>9) & 0x0001);
			ROTOR_GOVERNER_LIGHT((uiLightsFlag>>10) & 0x0001);
			ENG_FIRE_LIGHT((uiLightsFlag>>11) & 0x0001);			
			AUX_FUEL_PUMP_LIGHT((uiLightsFlag>>12) & 0x0001);
			FUEL_FILTER_LIGHT((uiLightsFlag>>13) & 0x0001);
		}
	}
}

/**
  * @brief  instrument pointer come back to zero.
  * @param  None
  * @retval None
  */
void InstrumentPointerToZero()
{
	for(int i=0;i<15;i++)
	{
		StepperConfig((eInstrumentSort)i,DirectionCCW,ENABLE);
	}
	StepperConfig(ATTITUDE_INDICATOR,DirectionCW,ENABLE);
	StepperConfig(ENGINE_TACH,DirectionCW,ENABLE);
	for(int i=0;i<1000;i++)
	{
		if((0==GPIO_ReadInputDataBit(ROLL_SENSOR_INPUT))&&(0==SensorFlag1))
		{
			Delay(50);
			if(0==GPIO_ReadInputDataBit(ROLL_SENSOR_INPUT)&&(0==SensorFlag1))
			{
				StepperConfig(ALTIMETER_INDICATOR,DirectionCW,DISABLE);
				StepperConfig(COMPASS_INDICATOR,DirectionCW,DISABLE);
				SensorFlag1=1;
			}
			else
			{
				//nothing
			}
		}
//		if(1==GPIO_ReadInputDataBit(COMPASS_SENSOR_INPUT))
//		{
//			StepperConfig(COMPASS_INDICATOR,DirectionCW,DISABLE);
//		}
		if(0==GPIO_ReadInputDataBit(ALTIMETER_SENSOR_INPUT)&&(0==SensorFlag2))
		{
			Delay(50);
			if(0==GPIO_ReadInputDataBit(ALTIMETER_SENSOR_INPUT)&&(0==SensorFlag2))
			{
				StepperConfig(CARBURETOR_TEMPERATURE,DirectionCW,DISABLE);
				StepperConfig(COMPASS_INDICATOR,DirectionCW,DISABLE);
				SensorFlag2=1;
			}
			else
			{
				//nothing
			}
		}
	}
	while((0==SensorFlag1)||(0==SensorFlag2))//(0!=GPIO_ReadInputDataBit(ROLL_SENSOR_INPUT))/*||(1!=GPIO_ReadInputDataBit(COMPASS_SENSOR_INPUT))*/||(0!=GPIO_ReadInputDataBit(ALTIMETER_SENSOR_INPUT))
	{
		if(0==GPIO_ReadInputDataBit(ROLL_SENSOR_INPUT)&&(0==SensorFlag1))
		{
			Delay(50);
			if(0==GPIO_ReadInputDataBit(ROLL_SENSOR_INPUT)&&(0==SensorFlag1))
			{
				StepperConfig(ALTIMETER_INDICATOR,DirectionCW,DISABLE);
				StepperConfig(COMPASS_INDICATOR,DirectionCW,DISABLE);
				SensorFlag1=1;
			}
			else
			{
				//nothing
			}
		}
//		if(1==GPIO_ReadInputDataBit(COMPASS_SENSOR_INPUT))
//		{
//			StepperConfig(COMPASS_INDICATOR,DirectionCW,DISABLE);
//		}
		if(0==GPIO_ReadInputDataBit(ALTIMETER_SENSOR_INPUT)&&(0==SensorFlag2))
		{
			Delay(50);
			if(0==GPIO_ReadInputDataBit(ALTIMETER_SENSOR_INPUT)&&(0==SensorFlag2))
			{
				StepperConfig(CARBURETOR_TEMPERATURE,DirectionCW,DISABLE);
				StepperConfig(COMPASS_INDICATOR,DirectionCW,DISABLE);
				SensorFlag2=1;
			}
		}
	}
	SensorFlag1=0;
	SensorFlag2=0;
	for(int i=0;i<15;i++)
	{
		StepperConfig((eInstrumentSort)i,DirectionCCW,DISABLE);
	}
}
/**
  * @brief  Configure the TIM3 Output Channels.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* GPIOC Configuration: TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8) and TIM3 CH4 (PC9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	
	/* Enable the TIM1 Trigger and commutation interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
	
	/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 1 PWM signals
		
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 21 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /21 MHz) - 1
                                              
    To get TIM3 output clock at 30 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM3 counter clock / TIM3 output clock) - 1
           = 665
                  
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */   


  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 3000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = (uint32_t)(3000000/100)-1;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channe1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = (uint32_t)(3000000/100/2);
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
	
	/* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void IndicatorLightInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOG Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF| RCC_AHB1Periph_GPIOC| RCC_AHB1Periph_GPIOE| \
												RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_GPIOD, ENABLE);
	
  /* Configure xxx as output push pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
                                
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	
  GPIO_Init(GPIOE, &GPIO_InitStructure);  
	
	/* Configure xxx as output push pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | \
																GPIO_Pin_14 | GPIO_Pin_15;
                                
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure xxx as output push pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11 | \
																GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
                                
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	
  GPIO_Init(GPIOD, &GPIO_InitStructure);  
	
	/* Configure xxx as output push pull*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2  | GPIO_Pin_3  | GPIO_Pin_4  | GPIO_Pin_5 | \
																GPIO_Pin_6  | GPIO_Pin_7;
                                
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	
  GPIO_Init(GPIOG, &GPIO_InitStructure);  
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
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

  while (1)
  {}
}
#endif
/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
