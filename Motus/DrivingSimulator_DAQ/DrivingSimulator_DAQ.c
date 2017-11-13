/**
  ******************************************************************************
  * @file    DrivingSimulator_DAQ.c
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
#include "DrivingSimulator_DAQ.h"
#include "MT_CommonFunction.h"
#include "stdio.h"
#include "main.h"
/** @addtogroup Motus_Driver
  * @{
  */

/** @defgroup DAQ_PCB 
  * @brief DAQ_PCB driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define AD_NUMBER_OF_TIMES                  (10)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup DAQ_PCB_Private_Functions
  * @{
  */ 

/** @defgroup DAQ_PCB_Group1 Initialization and Configuration
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
  * @brief  initializes the GPIOx peripheral registers.
  * @note   By default, The GPIO pins are configured in input floating mode (except JTAG pins).
  * @param  GPIOx: where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
  *                      x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
  *                      x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.  
  * @retval None
  */
void Digital_Input_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOA GPIOB GPIOC GPIOD GPIOE Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | \
													RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
	/* Configure pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//PA
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_8	|	GPIO_Pin_11	|	GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//PB
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_5	|	GPIO_Pin_6	|	GPIO_Pin_7	|	GPIO_Pin_8	|	GPIO_Pin_9	|	\
																GPIO_Pin_10	|	GPIO_Pin_11	|	GPIO_Pin_12	|	GPIO_Pin_13	|	GPIO_Pin_14	| \
																GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PC
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_6	|	GPIO_Pin_7	|	GPIO_Pin_8	|	GPIO_Pin_9	|	\
																GPIO_Pin_10	|	GPIO_Pin_11	|	GPIO_Pin_12	|	GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//PD
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_2	|	GPIO_Pin_3	|	GPIO_Pin_4	|	\
																GPIO_Pin_7	|	GPIO_Pin_8	|	GPIO_Pin_9	|	\
																GPIO_Pin_10	|	GPIO_Pin_11	|	GPIO_Pin_12	|	GPIO_Pin_13	|	GPIO_Pin_14	| \
																GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//PE
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0	|	GPIO_Pin_1	|	GPIO_Pin_2	|	GPIO_Pin_3	|	GPIO_Pin_4	|	\
																GPIO_Pin_5	|	GPIO_Pin_6	|	GPIO_Pin_7	|	GPIO_Pin_8	|	GPIO_Pin_9	|	\
																GPIO_Pin_10	|	GPIO_Pin_11	|	GPIO_Pin_12	|	GPIO_Pin_13	|	GPIO_Pin_14	| \
																GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void Diggital_Output_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/* GPIOG Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/* Configure */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14	| GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}
void Analog_Input_Init()
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	/* Configure pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//PA
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0	|	GPIO_Pin_1	|	GPIO_Pin_2	|	GPIO_Pin_3	|	GPIO_Pin_4	|	\
																GPIO_Pin_5	|	GPIO_Pin_6	|	GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//PB
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0	|	GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PC
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0	|	GPIO_Pin_1	|	GPIO_Pin_2	|	GPIO_Pin_3	|	GPIO_Pin_4	|	\
																GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
}

//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_AdcExpansion(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}


void LED_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  /* GPIOC Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* Configure PC14 and PC15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

unsigned char InputDebounce(unsigned char *t_value,GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	unsigned char t_InputValue=0;
	t_InputValue=GPIO_ReadInputDataBit(GPIOx,GPIO_Pin);
	if(*t_value == t_InputValue)
	{
		return t_InputValue;
	}
	else
	{
		*t_value=t_InputValue;
		return !t_InputValue;
	}
}

unsigned char PedalValueProcess(unsigned char adcInitialValue,unsigned char adcCurrentValue)
{
  short          t_Difference=0;
  const unsigned char  MinThrottle=110;
  const unsigned char  MinBrake=110;
  const unsigned char  MinClutch=110;
  const unsigned char MinPedalValue=110;
  t_Difference = adcInitialValue- adcCurrentValue;
  if(0>t_Difference)
  {
    return 0;
  }
  else
  {
    if(49<=t_Difference)
    {
      return 100;
    }
    else
    {
      return (t_Difference/0.5);
    }
  }
}

int DrivingSimulator_DAQ_Main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f40_41xxx.s/startup_stm32f427_437xx.s/startup_stm32f429_439xx.s)
       before to branch to application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
	unsigned int tADC_Value[16]={0};
	unsigned short ADC_Value[16]={0};
	unsigned short AdcInitialValue[16]={0};


	unsigned short Timing1=0;
	
	unsigned char ucSmallLigh=0;
	unsigned char ucLargeLight=0;
	unsigned char	unAutoLight=0;
	unsigned char	ucFarLight=0;
	unsigned char	ucShortLight=0;
	unsigned char	ucLeftTurnLight=0;
	unsigned char	ucRightTurnLight=0;
	unsigned char	ucFogLight=0;
	
	unsigned char	ucHorn=0;
	unsigned char	ucKeyAcc=0;
	unsigned char	ucKeyOn=0;
	unsigned char	ucKeyStart=0;
	unsigned char	ucWiperInt=0;
	unsigned char	ucWiperLo=0;
	unsigned char	ucWiperHi=0;
	unsigned char	ucSprayer=0;
	
	unsigned char ucBackWiperOnce=0;
	unsigned char	ucBackWiperOff=0;
	unsigned char	ucBackWiperInt=0;
	unsigned char	ucBackwiperOn=0;
	unsigned char	ucBackWiperSprayer=0;
	unsigned char	ucFirstGear=0;
	unsigned char ucSecondGear=0;
	unsigned char	ucThirdGear=0;
	
	unsigned char	ucFourthGear=0;
	unsigned char	ucFifthGear=0;
	unsigned char	ucReverseGear=0;
	unsigned char	ucSafetyBelt=0;
	unsigned char	ucHazardWarningLamp=0;
	unsigned char	ucReserve0=0;
	unsigned char	ucReserve1=0;
	unsigned char	ucReserve2=0;
	
	unsigned char	ucReserve3=0;
	unsigned char	ucReserve4=0;
	unsigned char	ucReserve5=0;
	unsigned char	ucReserve6=0;
	unsigned char	ucReserve7=0;
	unsigned char	ucReserve8=0;
	unsigned char	ucReserve9=0;
	unsigned char	ucReserve10=0;
	
	unsigned char	ucReserve11=0;
	unsigned char	ucReserve12=0;
	unsigned char	ucReserve13=0;
	unsigned char	ucReserve14=0;
	unsigned char	ucReserve15=0;
	unsigned char	ucReserve16=0;
	unsigned char	ucReserve17=0;
	unsigned char	ucReserve18=0;
	
	unsigned char t_ucSmallLigh=0;
	unsigned char t_ucLargeLight=0;
	unsigned char	t_unAutoLight=0;
	unsigned char	t_ucFarLight=0;
	unsigned char	t_ucShortLight=0;
	unsigned char	t_ucLeftTurnLight=0;
	unsigned char	t_ucRightTurnLight=0;
	unsigned char	t_ucFogLight=0;
	              
	unsigned char	t_ucHorn=0;
	unsigned char	t_ucKeyAcc=0;
	unsigned char	t_ucKeyOn=0;
	unsigned char	t_ucKeyStart=0;
	unsigned char	t_ucWiperInt=0;
	unsigned char	t_ucWiperLo=0;
	unsigned char	t_ucWiperHi=0;
	unsigned char	t_ucSprayer=0;
	
	unsigned char t_ucBackWiperOnce=0;
	unsigned char	t_ucBackWiperOff=0;
	unsigned char	t_ucBackWiperInt=0;
	unsigned char	t_ucBackwiperOn=0;
	unsigned char	t_ucBackWiperSprayer=0;
	unsigned char	t_ucFirstGear=0;
	unsigned char t_ucSecondGear=0;
	unsigned char	t_ucThirdGear=0;
	              
	unsigned char	t_ucFourthGear=0;
	unsigned char	t_ucFifthGear=0;
	unsigned char	t_ucReverseGear=0;
	unsigned char	t_ucSafetyBelt=0;
	unsigned char	t_ucHazardWarningLamp=0;
	unsigned char	t_ucReserve0=0;
	unsigned char	t_ucReserve1=0;
	unsigned char	t_ucReserve2=0;
	              
	unsigned char	t_ucReserve3=0;
	unsigned char	t_ucReserve4=0;
	unsigned char	t_ucReserve5=0;
	unsigned char	t_ucReserve6=0;
	unsigned char	t_ucReserve7=0;
	unsigned char	t_ucReserve8=0;
	unsigned char	t_ucReserve9=0;
	unsigned char	t_ucReserve10=0;
	              
	unsigned char	t_ucReserve11=0;
	unsigned char	t_ucReserve12=0;
	unsigned char	t_ucReserve13=0;
	unsigned char	t_ucReserve14=0;
	unsigned char	t_ucReserve15=0;
	unsigned char	t_ucReserve16=0;
	unsigned char	t_ucReserve17=0;
	unsigned char	t_ucReserve18=0;
  
  int iBreathLightTiming=0;
  CanTxMsg TxMessageForIO;
  CanTxMsg TxForAD_FirstSet;
  CanTxMsg TxForAD_SecondSet;
	/* NVIC configuration */
	NVIC_Config();
	/* Digital_Input configuration */
	Digital_Input_Init();
	/* ADC configuration */
	Analog_Input_Init();
	/* CAN configuration */
	CAN_BSP_Config();
	DebugComPort_Init(USART1);
	TIM_Configuration();
  LED_Init();
  /* Transmit Structure preparation */
  TxMessageForIO.StdId = 0x187;
  TxMessageForIO.ExtId = 0x00;
  TxMessageForIO.RTR = CAN_RTR_DATA;
  TxMessageForIO.IDE = CAN_ID_STD;
  TxMessageForIO.DLC = 8;
  
  TxForAD_FirstSet.StdId = 0x188;
  TxForAD_FirstSet.ExtId = 0x00;
  TxForAD_FirstSet.RTR = CAN_RTR_DATA;
  TxForAD_FirstSet.IDE = CAN_ID_STD;
  TxForAD_FirstSet.DLC = 8;
  
  TxForAD_SecondSet.StdId = 0x189;
  TxForAD_SecondSet.ExtId = 0x00;
  TxForAD_SecondSet.RTR = CAN_RTR_DATA;
  TxForAD_SecondSet.IDE = CAN_ID_STD;
  TxForAD_SecondSet.DLC = 8;
  
  DAQ_LED1_STATUS_SET(0);
  DAQ_LED2_STATUS_SET(1);
  //Get ADC Initial Value
	for(int i=0;i<AD_NUMBER_OF_TIMES;i++)
	{
    for(int j=0;j<16;j++)
    {
      tADC_Value[j]+=(Get_AdcExpansion(j)>>4);
    }
		Delay(10);
	}
  for(int i=0;i<16;i++)
  {
    AdcInitialValue[i]=180;//tADC_Value[i]/AD_NUMBER_OF_TIMES-5;
    tADC_Value[i]=0;
  }
	while(1)
	{
		if(1==Tim2_Flag)
		{
			Tim2_Flag=0;
      iBreathLightTiming++;
      if(50<=iBreathLightTiming)
      {
        iBreathLightTiming=0;
        DAQ_LED1_TOGGLE;
        DAQ_LED2_TOGGLE;
      }
			ucSmallLigh				  =InputDebounce(&t_ucSmallLigh,SMALL_LIGHT_SW_PIN);
			ucLargeLight			  =InputDebounce(&t_ucLargeLight,LARGE_LIGHT_SW_PIN);
			unAutoLight				  =InputDebounce(&t_unAutoLight,AUTO_LIGHT_SW_PIN);
			ucFarLight				  =InputDebounce(&t_ucFarLight,FAR_LIGHT_SW_PIN);
			ucShortLight			  =InputDebounce(&t_ucShortLight,SHORT_LIGHT_SW_PIN);
			ucLeftTurnLight		  =InputDebounce(&t_ucLeftTurnLight,LEFT_TURN_LIGHT_SW_PIN);
			ucRightTurnLight	  =InputDebounce(&t_ucRightTurnLight,RIGHT_TURN_LIGHT_SW_PIN);
			ucFogLight				  =InputDebounce(&t_ucFogLight,FOG_LIGHT_SW_PIN);
			TxMessageForIO.Data[0]= (ucFogLight<<7)|(ucRightTurnLight<<6)|(ucLeftTurnLight<<5)|(ucShortLight<<4)| \
                              (ucFarLight<<3)|(unAutoLight<<2)|(ucLargeLight<<1)|(ucSmallLigh<<0);
      
			ucHorn						  =InputDebounce(&t_ucHorn,HORN_SW_PIN);
			ucKeyAcc					  =InputDebounce(&t_ucKeyAcc,KEY_ACC_STATE_PIN);
			ucKeyOn						  =InputDebounce(&t_ucKeyOn,KEY_ON_STATE_PIN);
			ucKeyStart				  =InputDebounce(&t_ucKeyStart,KEY_START_STATE_PIN);
			ucWiperInt				  =InputDebounce(&t_ucWiperInt,WIPER_INT_STATE_PIN);
			ucWiperLo					  =InputDebounce(&t_ucWiperLo,WIPER_LO_STATE_PIN);
			ucWiperHi					  =InputDebounce(&t_ucWiperHi,WIPER_HI_STATE_PIN);
			ucSprayer					  =InputDebounce(&t_ucSprayer,SPRAYER_SW_PIN);
			TxMessageForIO.Data[1]= (ucSprayer<<7)|(ucWiperHi<<6)|(ucWiperLo<<5)|(ucWiperInt<<4)| \
                              (ucKeyStart<<3)|(ucKeyOn<<2)|(ucKeyAcc<<1)|(ucHorn<<0);
      
			ucBackWiperOnce		  =InputDebounce(&t_ucBackWiperOnce,BACK_WIPER_ONCE_STATE_PIN);
			ucBackWiperOff		  =InputDebounce(&t_ucBackWiperOff,BACK_WIPER_OFF_STATE_PIN);
			ucBackWiperInt		  =InputDebounce(&t_ucBackWiperInt,BACK_WIPER_INT_STATE_PIN);
			ucBackwiperOn			  =InputDebounce(&t_ucBackwiperOn,BACK_WIPER_ON_STATE_PIN);
			ucBackWiperSprayer  =InputDebounce(&t_ucBackWiperSprayer,BACK_WIPER_SPRAYER_STATE_PIN);
			ucFirstGear				  =InputDebounce(&t_ucFirstGear,FIRST_GEAR_PIN);
			ucSecondGear			  =InputDebounce(&t_ucSecondGear,SECOND_GEAR_PIN);
			ucThirdGear				  =InputDebounce(&t_ucThirdGear,THIRD_GEAR_PIN);
			TxMessageForIO.Data[2]= (ucThirdGear<<7)|(ucSecondGear<<6)|(ucFirstGear<<5)|(ucBackWiperSprayer<<4)| \
                              (ucBackwiperOn<<3)|(ucBackWiperInt<<2)|(ucBackWiperOff<<1)|(ucBackWiperOnce<<0);
      
			ucFourthGear			  =InputDebounce(&t_ucFourthGear,FOURTH_GEAR_PIN);
			ucFifthGear         =InputDebounce(&t_ucFifthGear,FIFTH_GEAR_PIN);
			ucReverseGear       =InputDebounce(&t_ucReverseGear,REVERSE_GEAR_PIN);
			ucSafetyBelt        =InputDebounce(&t_ucSafetyBelt,SAFETY_BELT_PIN);
			ucHazardWarningLamp =InputDebounce(&t_ucHazardWarningLamp,HAZARD_WARNING_LAMP_PIN);
			ucReserve0          =InputDebounce(&t_ucReserve0,RESERVE0_PIN);
			ucReserve1          =InputDebounce(&t_ucReserve1,RESERVE1_PIN);
			ucReserve2          =InputDebounce(&t_ucReserve2,RESERVE2_PIN);
			TxMessageForIO.Data[3]= (ucReserve2<<7)|(ucReserve1<<6)|(ucReserve0<<5)|(ucHazardWarningLamp<<4)| \
                              (ucSafetyBelt<<3)|(ucReverseGear<<2)|(ucFifthGear<<1)|(ucFourthGear<<0);
      
			ucReserve3          =InputDebounce(&t_ucReserve3,RESERVE3_PIN);
			ucReserve4          =InputDebounce(&t_ucReserve4,RESERVE4_PIN);
			ucReserve5          =InputDebounce(&t_ucReserve5,RESERVE5_PIN);
			ucReserve6          =InputDebounce(&t_ucReserve6,RESERVE6_PIN);
			ucReserve7          =InputDebounce(&t_ucReserve7,RESERVE7_PIN);
			ucReserve8          =InputDebounce(&t_ucReserve8,RESERVE8_PIN);
			ucReserve9          =InputDebounce(&t_ucReserve9,RESERVE9_PIN);
			ucReserve10         =InputDebounce(&t_ucReserve10,RESERVE10_PIN);
			TxMessageForIO.Data[4]= (ucReserve10<<7)|(ucReserve9<<6)|(ucReserve8<<5)|(ucReserve7<<4)| \
                              (ucReserve6<<3)|(ucReserve5<<2)|(ucReserve4<<1)|(ucReserve3<<0);
      
			ucReserve11         =InputDebounce(&t_ucReserve11,RESERVE11_PIN);
			ucReserve12         =InputDebounce(&t_ucReserve12,RESERVE12_PIN);
			ucReserve13         =InputDebounce(&t_ucReserve13,RESERVE13_PIN);
			ucReserve14         =InputDebounce(&t_ucReserve14,RESERVE14_PIN);
			ucReserve15         =InputDebounce(&t_ucReserve15,RESERVE15_PIN);
			ucReserve16         =InputDebounce(&t_ucReserve16,RESERVE16_PIN);
			ucReserve17         =InputDebounce(&t_ucReserve17,RESERVE17_PIN);
			ucReserve18         =InputDebounce(&t_ucReserve18,RESERVE18_PIN);
      TxMessageForIO.Data[5]= (ucReserve18<<7)|(ucReserve17<<6)|(ucReserve16<<5)|(ucReserve15<<4)| \
                              (ucReserve14<<3)|(ucReserve13<<2)|(ucReserve12<<1)|(ucReserve11<<0);
      CAN_Transmit(CAN1, &TxMessageForIO);
      
      Timing1++;
			for(int i=0;i<16;i++)
			{
				tADC_Value[i]+=(Get_AdcExpansion(i)>>4);
			}
			if(AD_NUMBER_OF_TIMES==Timing1)
			{
				Timing1=0;
				for(int i=0;i<16;i++)
				{
					ADC_Value[i]=tADC_Value[i]/AD_NUMBER_OF_TIMES;
					tADC_Value[i]=0;
				}
        for(int i=0;i<8;i++)
        {
          TxForAD_FirstSet.Data[i]=ADC_Value[i];
        }
        TxForAD_FirstSet.Data[1]=PedalValueProcess(180,TxForAD_FirstSet.Data[1]);//acc
        CAN_Transmit(CAN1, &TxForAD_FirstSet);
        
        for(int i=0;i<8;i++)
        {
          TxForAD_SecondSet.Data[i]=ADC_Value[i+8];
        }
				if(50<TxForAD_SecondSet.Data[PARKING_BRAKE-8])
				{
					TxForAD_SecondSet.Data[PARKING_BRAKE-8]=1;
				}
				else
				{
					TxForAD_SecondSet.Data[PARKING_BRAKE-8]=0;
				}
        TxForAD_SecondSet.Data[CLUTCH-8]=100-PedalValueProcess(180,TxForAD_SecondSet.Data[CLUTCH-8]);
        TxForAD_SecondSet.Data[BRAKE-8]=PedalValueProcess(180,TxForAD_SecondSet.Data[BRAKE-8]);
        CAN_Transmit(CAN1, &TxForAD_SecondSet);
				printf("TxMessageForIO.Data[0] is %4x;\r\n",TxMessageForIO.Data[0]);
				printf("TxMessageForIO.Data[1] is %4x;\r\n",TxMessageForIO.Data[1]);
        printf("TxMessageForIO.Data[2] is %4x;\r\n",TxMessageForIO.Data[2]);
        printf("TxMessageForIO.Data[3] is %4x;\r\n",TxMessageForIO.Data[3]);
				printf("TxMessageForIO.Data[4] is %4x;\r\n",TxMessageForIO.Data[4]);
				printf("TxMessageForIO.Data[5] is %4x;\r\n",TxMessageForIO.Data[5]);
				printf("ADC_Value[WIPER_INT_TIME] is %4d;\r\n",ADC_Value[WIPER_INT_TIME]);
				printf("ADC_Value[PARKING_BRAKE] is %4d;\r\n",ADC_Value[PARKING_BRAKE]);
				
				printf("ADC_Value[CLUTCH] is %4d;\r\n",ADC_Value[CLUTCH]);
				printf("ADC_Value[BRAKE] is %4d;\r\n",ADC_Value[BRAKE]);
				printf("ADC_Value[ACCELERATOR_PEDAL] is %4d;\r\n",ADC_Value[ACCELERATOR_PEDAL]);
				printf("ADC_Value[INDOOR_REARVIEW_X_AXIS] is %4d;\r\n",ADC_Value[INDOOR_REARVIEW_X_AXIS]);
				printf("ADC_Value[INDOOR_REARVIEW_Y_AXIS] is %4d;\r\n",ADC_Value[INDOOR_REARVIEW_Y_AXIS]);
			}
		}
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

/************************ (C) COPYRIGHT MOTUS *****END OF FILE****/
