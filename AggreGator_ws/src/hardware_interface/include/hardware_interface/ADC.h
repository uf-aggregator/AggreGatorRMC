#ifndef ADC_H
#define ADC_H

//Includes
#include "i2c.h"

//Enumerators

//ADC enumerator (Addresses of the ADCs)
enum ADC
{
    ADC_CURRENT_SENSE   = 0x1D,
    ADC_IR              = 0x35
};

//Register enumerator
enum ADC_REG
{
    ADC_CONFIG          = 0,
    ADC_INT             = 1,
    ADC_INT_MASK        = 3,
    ADC_CONVERSION_RATE = 7,
    ADC_CHANNEL_DISABLE = 8,
    ADC_ONE_SHOT        = 9,
    ADC_DEEP_SHUTDOWN   = 0xA,
    ADC_ADV_CONFIG      = 0xB,
    ADC_BUSY            = 0xC,

    //Channel registers (keep in order they are sequintial)
    ADC_CH0             = 0x20,
    ADC_CH1,
    ADC_CH2,
    ADC_CH3,
    ADC_CH4,
    ADC_CH5,
    ADC_CH6,
    ADC_CH7,

    //Limit registers (keep in order they are sequintial)
    ADC_LIMIT_0         = 0x2A,
    ADC_LIMIT_1,
    ADC_LIMIT_2,
    ADC_LIMIT_3,
    ADC_LIMIT_4,
    ADC_LIMIT_5,
    ADC_LIMIT_6,
    ADC_LIMIT_7


};

/*
 * notes
 *
 * Writes are composed of ADC address, register address, data to write
 *
 * Reads:
 *      First if not done previously write the ADC address then imideatly write the address of the register
 *      Then read from the ADC as you would normally (write the address out)
 *
 * Digital output = [(Inx - GND) / VREF] * 2^12
 */

/*
 * Read ADC
 *
 * Read ADC number adc
 */
u_int16_t ReadADC(ADC adc, ADC_REG reg);

/*
 * Initilise all adcs
 *
 * Returns true if succsful false if unsuccesfull
 */
bool InitADC(ADC adc);

/*
 * Check the busy flag
 *
 * Poll for the busy flag returns true if busy, false if not
 */
bool isADCBusy();

#endif // ADC_H
