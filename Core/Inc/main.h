#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "init.h"
#include "stdbool.h"

#define PERIOD 480
#define UPPER 432
#define LOWER 0

#define kp 1
#define ki 0.1
#define ERROR_GAIN 0.001

float dac_duty, pwm_duty, vg; 
uint32_t dac_value, pwm_value;

bool startup;
uint32_t ADC_value[8];
uint32_t pid(uint32_t set, uint32_t value);






#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
