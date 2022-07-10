#include "main.h"

float I_CELL;
int ADCValue, ADCValuesum;
int sensor;
int dv;
int ref, refsum;
int final, count, count1;
int first, second;
float dI;
#define kp 1
#define ki 1
float error;

#define I_SET 150

int main(void)
{
  dac = 0;
  dac_duty = 0;
  pulse = 0;
  duty = 0;
  vg = 0;
  final = 0;
  count = 0;
  count1 = 0;
  ADCValuesum = 0;
  first = 0; 
  second = 0;
  refsum = 0;
  init();
  
  while (1)
  {
    

	pulse = (duty * 480) / 100;
        dac_duty = vg * 100 / (4.95);
                dac = (dac_duty * 480) / 100;
        update_DAC(dac);
//         
  
        update_PWM(pulse);
        
        HW_Check();
        if (!first)
        {
          ref = update_ADC()*1000;
          refsum = ref + refsum;
          count1 ++;
          if(count1 > 100)
          {
            ref = (refsum)/(count1*0.604);
            first = 1;
          }
          
        }
        
        ADCValue = update_ADC()*1000;
        ADCValuesum = ADCValue + ADCValuesum ;
        count++;
        
        if(count > 100)
        {
          final = ADCValuesum/count;
          count = 0;
          ADCValuesum = 0;
          second = 1;
        }
        sensor = final/0.604;
        dv = (ref - sensor);
        I_CELL = dv/0.1;
        
        /*
        if (first && second)
        {
        dI = (I_SET - I_CELL)/10;
        error += dI;
        dac_duty += (kp*dI) + (ki*error);
        if (dac_duty >= 100)
        {
          dac_duty = 100;
          dI = 0;
          error = 0;
        }
        else if (dac_duty < 0)
        {
          dac_duty = 0;
        }
        dac = (dac_duty * 480) / 100;
        update_DAC(dac);
  }
*/
  }
}




#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif 

