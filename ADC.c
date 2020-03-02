#ifndef __ADC_C
#define __ADC_C////////////////////////////////////////

#include "define.h"
extern eeprom BYTE SensorType[MAX_SEN_NUM];//= {0, 0, 0, 0, 0, 0, 0, 0};
extern eeprom char ADCMux[MAX_SEN_NUM];
extern BYTE objToMsr;

// Read the AD conversion result
unsigned int ReadAdc()
{


    if (objToMsr == BATTERY)
       ADMUX = BATTERY_MUX | ADC_VREF_TYPE;     //V6  =  7
     else
    {
        ADMUX = ADCMux[objToMsr] | ADC_VREF_TYPE;
    }
    // Delay needed for the stabilization of the ADC input voltage
    delay_us(10);
    // Start the AD conversion
    ADCSRA|=(1<<ADSC);
    // Wait for the AD conversion to complete
    while ((ADCSRA & (1<<ADSC))==0);

    ADCSRA|=(1<<ADIF);   //CLR INT FLAG - NOT USED HERE
    return ADCW;

}

unsigned int AnalogSensRead()
{
    char i;
    unsigned int adc_res, hi_value, lo_value;
    unsigned long cur_value;


//    #ifdef DebugMode
//        SendDebugMsg("\r\nAnalogSensRead()\r\n");
//        #endif DebugMode

    // enable ADC conversion
   // ADCSRA|=(1<<ADEN);       //ENABLE ADC
    ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    PRR0_EN_ADC();

    hi_value = adc_res = 0;
    lo_value = 0xFFFF;

    for(i = 0; i < 10; i++)
    {
        cur_value = ReadAdc();

        // calculate the voltage in [mV]: io 3 & 4 - ext, all other - internal
      //    cur_value = (cur_value * INT_VREF)/1024L; // 2560 mV ref
         cur_value = cur_value * 2.56; // 2560 mV ref 
     //   cur_value = (cur_value * VCC_VREF)/1024L;   // 3300 mV ref

        delay_ms(5);
        if (hi_value < cur_value)
            hi_value = (int)cur_value;
        if (lo_value > cur_value)
            lo_value = (int)cur_value;
        adc_res += cur_value;
    }
    adc_res = adc_res - hi_value;
    adc_res = adc_res - lo_value;
    adc_res = adc_res / 8;

         if (objToMsr == BATTERY)
         {
                SendDebugMsg("\r\nBattery val= ");
                PrintNum(adc_res*4.7);             
         }


   //  CLRBIT(ADCSRA,ADEN);  //DISABLE ADC
     ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    return adc_res;
}

unsigned int STR2BIN(unsigned char *str)
 {

  unsigned int BinTmp;

      BinTmp = (unsigned int)(str[4]-0x30);
      BinTmp += (unsigned int)(str[3]-0x30)*10;
      BinTmp += (unsigned int)(str[2]-0x30)*100;
      BinTmp += (unsigned int)(str[1]-0x30)*1000;
      BinTmp += (unsigned int)(str[0]-0x30)*10000;

      return BinTmp;
  }
#endif __ADC_C
