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
float update_ADC(void);
void HW_Check(void);


void Error_Handler(void);



#endif /* __INIT_H_ */