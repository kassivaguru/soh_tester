#ifndef __INIT_H_
#define __INIT_H_

#include "stm32f0xx_hal.h"

#define USER_SW_1_Pin GPIO_PIN_0
#define USER_SW_1_GPIO_Port GPIOF
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOF
#define RS485_EN_Pin GPIO_PIN_0
#define RS485_EN_GPIO_Port GPIOB
#define PRECHG_EN_Pin GPIO_PIN_11
#define PRECHG_EN_GPIO_Port GPIOA
#define INPUT_EN_Pin GPIO_PIN_12
#define INPUT_EN_GPIO_Port GPIOA
#define CELL_EN_Pin GPIO_PIN_15
#define CELL_EN_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_3
#define LED_GREEN_GPIO_Port GPIOB
#define LED_DEBUG_Pin GPIO_PIN_4
#define LED_DEBUG_GPIO_Port GPIOB
#define DCH_OVRD_Pin GPIO_PIN_6
#define DCH_OVRD_GPIO_Port GPIOB
#define BLEED_EN_Pin GPIO_PIN_7
#define BLEED_EN_GPIO_Port GPIOB

uint32_t Analog_value[8];
uint32_t ICELL_REF;

#define VOLTAGE_CELL		Analog_value[0]
#define VOLTAGE_INPUT		Analog_value[2]
#define VOLTAGE_CAT			Analog_value[4]
#define VOLTAGE_AN			Analog_value[5]
#define CURRENT_CELL		Analog_value[1]
#define CURRENT_INPUT		Analog_value[3]
#define TEMP_CELL			Analog_value[6]
#define TEMP_DCH_FET		Analog_value[7]

#define VOLTAGE_CELL_MCU		ADC_voltage[0]
#define VOLTAGE_INPUT_MCU		ADC_voltage[2]
#define VOLTAGE_CAT_MCU			ADC_voltage[4]
#define VOLTAGE_AN_MCU			ADC_voltage[5]
#define CURRENT_CELL_MCU		ADC_voltage[1]
#define CURRENT_INPUT_MCU		ADC_voltage[3]
#define TEMP_CELL_MCU			ADC_voltage[6]
#define TEMP_DCH_FET_MCU		ADC_voltage[7]

#define VREF			3300
#define VREF_TEMP		3300
#define BETA           	3950
#define CONST_RES      	10000

#define VIN_GAIN		4.0952
#define VIN_OFFSET		0

#define IIN_GAIN		3.33
#define IIN_OFFSET		0

#define VCELL_GAIN		1.667
#define VCELL_OFFSET	0

#define ICELL_GAIN		1
#define ICELL_GAIN_REF	1.655
#define ICELL_OFFSET	0

#define VCAT_GAIN		1.667
#define VCAT_OFFSET		0

#define VAN_GAIN		1.667
#define VAN_OFFSET		0

#define TCELL_GAIN		0
#define TCELL_OFFSET	0

#define TFET_GAIN		0
#define TFET_OFFSET		0

bool startup;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM16_Init(void);
void MX_ADC_Init(void);
void MX_USART1_UART_Init(void);

void init(void);
void update_PWM(int value);
void update_DAC(int value);

void update_ADC(void);

void HW_Check(void);


void Error_Handler(void);



#endif /* __INIT_H_ */