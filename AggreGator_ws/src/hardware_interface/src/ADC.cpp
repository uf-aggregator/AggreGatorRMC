#include "ADC.h"

/*
 * Check the busy flag
 *
 * Poll for the busy flag returns true if busy, false if not
 */
bool isADCBusy(ADC adc)
{
    return readfromreg_i2c(adc, ADC_BUSY, 1);
}

/*
 * Read ADC
 *
 * Read ADC number adc
 */
u_int16_t ReadADC(ADC adc, ADC_REG reg)
{
    //Read 2 bytes from register reg on ADC adc
    char[2] value = readfromreg_i2c(adc, reg, 2);

    //convert to a unsigned 16 bit integer
    u_int16_t output = value[0] + (value[1] << 8);
    return output;
}

/*
 * Initilise all adcs
 *
 * Returns true if succsful false if unsuccesfull
 * This function makes the assumption that 33ms have elapsed since the ADC was powered up
 */
bool InitADC()
{
    while(isADCBusy(ADC_IR)); //Wait until IR ADC isn't busy
    while(isADCBusy(ADC_CURRENT_SENSE)); //Wait until Current Sense ADC isn't busy

    //Set up config, shudown mode, interupts disabled and cleared
    writetoreg_i2c(ADC_IR,            ADC_CONFIG, 1, 0b00001000);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CONFIG, 1, 0b00001000);

    //Turn off all interupts
    writetoreg_i2c(ADC_IR,            ADC_INT_MASK, 1, 0xFF);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_INT_MASK, 1, 0xFF);

    //Set conversion rate to continous (WARNING: most be done in shutdown mode)
    writetoreg_i2c(ADC_IR,            ADC_CONVERSION_RATE, 1, 0x01);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CONVERSION_RATE, 1, 0x01);

    //Set channel conversions (WARNING: most be done in shutdown mode)
    writetoreg_i2c(ADC_IR,            ADC_CHANNEL_DISABLE, 1, 0x00);        //Turn on all IR's
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CHANNEL_DISABLE, 1, 0xFC);        //Turn on ch0 and 1

    //Set the advance config, use internal 2.56 V ref, mode 0
    writetoreg_i2c(ADC_IR,            ADC_ADV_CONFIG, 1, 0x00);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_ADV_CONFIG, 1, 0x00);

    //Turn on conversion
    writetoreg_i2c(ADC_IR,            ADC_CONFIG, 1, 0x09);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CONFIG, 1, 0x09);

    return true;
}
