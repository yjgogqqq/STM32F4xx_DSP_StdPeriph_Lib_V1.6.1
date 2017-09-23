/**
  ******************************************************************************
  * @file FullHalfStepMode/inc/StepperMotor.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Header file for StepperMotor.c
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


/* Define to prevent recursive inclusion ---------------------------------------*/
#ifndef __STEPPERMOTOR_H
#define __STEPPERMOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#define   TIM2_ARR_Address    0x4000002C
#define   BufferSize  20  
#define   Stepper_RotationDirection_CW     0x0001
#define   Stepper_RotationDirection_CCW    0x0002
#define   Stepper_Full    0x0003
#define   Stepper_Half    0x0004
#define   Stepper_ControlFast   0x0005
#define   Stepper_ControlSlow   0x0006


#define		VERTICAL_SPEED_STEPPER_DIR_PIN									GPIOF, GPIO_Pin_8
#define		VERTICAL_SPEED_STEPPER_ENABLE_PIN								GPIOF, GPIO_Pin_7

#define		ATTITUDE_INDICATOR_STEPPER_DIR_PIN							GPIOF, GPIO_Pin_6
#define		ATTITUDE_INDICATOR_STEPPER_ENABLE_PIN						GPIOF, GPIO_Pin_5
                                                          
#define		AIRSPEED_INDICATOR_STEPPER_DIR_PIN							GPIOF, GPIO_Pin_4
#define		AIRSPEED_INDICATOR_STEPPER_ENABLE_PIN						GPIOF, GPIO_Pin_3
                                                          
#define		ENGINE_TACH_STEPPER_DIR_PIN											GPIOF, GPIO_Pin_2
#define		ENGINE_TACH_STEPPER_ENABLE_PIN									GPIOF, GPIO_Pin_1
                                                          
#define		ROTOR_TACH_STEPPER_DIR_PIN											GPIOF, GPIO_Pin_0
#define		ROTOR_TACH_STEPPER_ENABLE_PIN										GPIOC, GPIO_Pin_15
                                                          
#define		ALTIMETER_INDICATOR_STEPPER_DIR_PIN							GPIOC, GPIO_Pin_14
#define		ALTIMETER_INDICATOR_STEPPER_ENABLE_PIN					GPIOC, GPIO_Pin_13
                                                          
#define		COMPASS_INDICATOR_STEPPER_DIR_PIN								GPIOE, GPIO_Pin_6
#define		COMPASS_INDICATOR_STEPPER_ENABLE_PIN						GPIOE, GPIO_Pin_5
                                                          
#define		MANIFOLD_PRESSURE_STEPPER_DIR_PIN								GPIOE, GPIO_Pin_4
#define		MANIFOLD_PRESSURE_STEPPER_ENABLE_PIN						GPIOE, GPIO_Pin_3
                                                          
#define		AMMETER_INDICATOR_STEPPER_DIR_PIN								GPIOE, GPIO_Pin_2
#define		AMMETER_INDICATOR_STEPPER_ENABLE_PIN						GPIOE, GPIO_Pin_1
                                                          
#define		OIL_PRESSURE_STEPPER_DIR_PIN										GPIOE, GPIO_Pin_0
#define		OIL_PRESSURE_STEPPER_ENABLE_PIN									GPIOB, GPIO_Pin_9
                                                          
#define		AUX_Fuel_Quantity_STEPPER_DIR_PIN								GPIOB, GPIO_Pin_8
#define		AUX_Fuel_Quantity_STEPPER_ENABLE_PIN						GPIOB, GPIO_Pin_7
                                                          
#define		OIL_TEMPERATURE_STEPPER_DIR_PIN									GPIOB, GPIO_Pin_6
#define		OIL_TEMPERATURE_STEPPER_ENABLE_PIN							GPIOB, GPIO_Pin_5
                                                          
#define		MAIN_FUEL_QUANTITY_STEPPER_DIR_PIN							GPIOG, GPIO_Pin_15	//GPIOB, GPIO_Pin_4
#define		MAIN_FUEL_QUANTITY_STEPPER_ENABLE_PIN						GPIOG, GPIO_Pin_14	//GPIOB, GPIO_Pin_3
                                                          
#define		CYLINDER_HEAD_TEMPERATURE_STEPPER_DIR_PIN				GPIOG, GPIO_Pin_13
#define		CYLINDER_HEAD_TEMPERATURE_STEPPER_ENABLE_PIN		GPIOG, GPIO_Pin_12

#define		CARBURETOR_TEMPERATURE_STEPPER_DIR_PIN					GPIOG, GPIO_Pin_11
#define		CARBURETOR_TEMPERATURE_STEPPER_ENABLE_PIN				GPIOG, GPIO_Pin_10

#define		ROTOR_CLUTCH_LIGHT_PIN													GPIOE, GPIO_Pin_12
#define		MR_TEMP_LIGHT_PIN																GPIOE, GPIO_Pin_13
#define		MR_CHIP_LIGHT_PIN																GPIOE, GPIO_Pin_14
#define		ROTOR_BRAKE_LIGHT_PIN														GPIOE, GPIO_Pin_15
#define		STARTER_LIGHT_PIN																GPIOB, GPIO_Pin_10
#define		TR_CHIP_LIGHT_PIN																GPIOB, GPIO_Pin_11
#define		LOW_FUEL_LIGHT_PIN															GPIOB, GPIO_Pin_12
#define		LOW_ROTOR_RPM_LIGHT_PIN													GPIOB, GPIO_Pin_13
#define		ALTERNATOR_LIGHT_PIN														GPIOB, GPIO_Pin_14
#define		LOW_OIL_PRESSURE_LIGHT_PIN											GPIOB, GPIO_Pin_15
#define		ROTOR_GOVERNER_LIGHT_PIN												GPIOD, GPIO_Pin_8
#define		ENG_FIRE_LIGHT_PIN															GPIOD, GPIO_Pin_9
#define		AUX_FUEL_PUMP_LIGHT_PIN													GPIOD, GPIO_Pin_10
#define		FUEL_FILTER_LIGHT_PIN														GPIOD, GPIO_Pin_11

#define		ROLL_SENSOR_INPUT																GPIOB, GPIO_Pin_3
#define		COMPASS_SENSOR_INPUT														GPIOB, GPIO_Pin_4
#define		ALTIMETER_SENSOR_INPUT													GPIOA, GPIO_Pin_15
extern int				nVerticalSpeedStepperCurPos						;
extern int				nAttitudeStepperCurPos								;
extern int				nAirSpeedStepperCurPos								;
extern int				nEngineTachSteppperCurPos							;
extern int				nRotorTachSteppperCurPos							;
extern int				nAltimeterSteppperCurPos							;
extern int				nCompassSteppperCurPos								;
extern int				nManifoldPressureSteppperCurPos				;
extern int				nAmmeterSteppperCurPos								;
extern int				nOilPressureSteppperCurPos						;
extern int				nAuxFuelQuantitySteppperCurPos				;
extern int				nOilTemperatureSteppperCurPos					;
extern int				nMainFuelQuantitySteppperCurPos				;
extern int				nCylinderHeadTemperatureSteppperCurPos;
extern int				nCarburetorTemperatureSteppperCurPos	;
extern int				nAirTemperatureSteppperCurPos					;
extern unsigned int			uiLightsFlag										;

extern int				nVerticalSpeedStepperDstPos						;
extern int				nAttitudeStepperDstPos								;
extern int				nAirSpeedStepperDstPos								;
extern int				nEngineTachSteppperDstPos							;
extern int				nRotorTachSteppperDstPos							;
extern int				nAltimeterSteppperDstPos							;
extern int				nCompassSteppperDstPos								;
extern int				nManifoldPressureSteppperDstPos				;
extern int				nAmmeterSteppperDstPos								;
extern int				nOilPressureSteppperDstPos						;
extern int				nAuxFuelQuantitySteppperDstPos				;
extern int				nOilTemperatureSteppperDstPos					;
extern int				nMainFuelQuantitySteppperDstPos				;
extern int				nCylinderHeadTemperatureSteppperDstPos;
extern int				nCarburetorTemperatureSteppperDstPos	;

extern uint8_t		PulseFlag;
extern u8 CanRevFlag;
/* Exported types --------------------------------------------------------------*/
/* VelocityProfile Init structure definition */
typedef struct
{ 
	uint16_t Stepper_MaximumSpeed;
	uint16_t Stepper_MinimumSpeed;
	uint16_t Stepper_SlewPeriod;
	uint16_t Stepper_Profile_TotalStepNumber;
} VelocityProfile_InitTypeDef; 
typedef enum
{
	VERTICAL_SPEED_INDICATOR,
	ATTITUDE_INDICATOR,
	AIRSPEED_INDICATOR,
	ENGINE_TACH,
	ROTOR_TACH,
	ALTIMETER_INDICATOR,
	COMPASS_INDICATOR,
	MANIFOLD_PRESSURE_INDICATOR,
	AMMETER_INDICATOR,
	OIL_PRESSURE,
	AUX_Fuel_Quantity,
	OIL_TEMPERATURE,
	MAIN_FUEL_QUANTITY,
	CYLINDER_HEAD_TEMPERATURE,
	CARBURETOR_TEMPERATURE,	
} eInstrumentSort;
typedef enum
{
	DirectionCW,
	DirectionCCW,
} eStepperRotationDirection;
/* Exported constants ---------------------------------------------------------*/     

/* Module private variables --------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Stepper_ResetDisable(void);
void Stepper_SetRotationDirection(uint16_t Stepper_RotationDirection);
void Stepper_SelectMode(uint16_t Stepper_Mode);
void Stepper_SetControlMode(uint16_t StepperControlMode);
void Stepper_Start(FunctionalState NewState);

void Stepper_PinControlConfig(void);
void Stepper_Cmd(FunctionalState NewState);
void StepperInit(void);
void Stepper_Init(void);
void StepperConfig(eInstrumentSort eInstrument,eStepperRotationDirection eDirection,FunctionalState NewState);
void StepperHandler(eInstrumentSort teInstrument,int *tipCurPos,int *tipDstPos);
/* Exported constants --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32_STEP_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

