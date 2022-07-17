#include "main.h"

#define I_SET 150 
#define V_SET 5000
#define P_SET 500

/*********/

startup = true;

int main(void)
{
  static pwm_duty = 0, dac_duty = 0, vg = 0; 
  init();
  updateADC();
  if(startup)
  {
    ICELL_REF = CURRENT_CELL;
    startup = 0;
  }
  while (1)
  {
    if(!startup)
    {
      HW_Check();
      pwm_value = (pwm_duty * 480) / 100;
      dac_duty = vg * 100 / (4.95);
     // dac_value = (dac_duty * 480) / 100;
      dac_value = pid(I_SET, CURRENT_CELL);
     // pwm_value = pid(V_SET, VOLTAGE_AN);
      update_DAC(dac_value);     
     // update_PWM(pwm_value); 
    }
  }
}

uint32_t pid(uint32_t set, uint32_t value)
{
  static float error = 0, dx = 0;
  uint32_t ctrl = 0;
  
  dx = (set - value) * ERROR_GAIN ;
  error += dx;
  ctrl += (kp*dx) + (ki*error);
  
  if ( ctrl >= PERIOD)
  {
    ctrl = UPPER;
  }
  else if (ctrl <= LOWER)
  {
    ctrl = LOWER;
  }
  else
  {
    ctrl = 0;
  }
  return ctrl;
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif 

