#include "ADC.h"

/*
 * Check the busy flag
 *
 * Poll for the busy flag returns true if busy, false if not
 */
bool isADCBusy(ADC adc)
{
    char* busy_reg = NULL;
    busy_reg = readfromreg_i2c(adc, ADC_BUSY, 1);
    if(busy_reg[0] == 1)
    {
        //Error handeling... Dirty bit set
    }

    if(busy_reg[1])
    {
        //Still busy
        free(busy_reg);
        return true;
    }
    else
    {
        //Free!!!!
        free(busy_reg);
        return false;
    }
}

/*
 * Read ADC
 *
 * Read ADC number adc
 */
u_int16_t ReadADC(ADC adc, ADC_REG reg)
{
    //Wait for ADC to not be busy
    while(isADCBusy(adc));

    //Read 2 bytes from register reg on ADC adc
    char* value = NULL;
    value = readfromreg_i2c(adc, reg, 2);

    //Check dirty bit
    if(value[0] == 1)
    {
        //Error handeling... Dirty bit set
    }

    //convert to a unsigned 16 bit integer
    u_int16_t output = (value[1] << 8) + value[2];

    //clean up and return
    free(value);
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
    //Initilize i2c
    init_i2c();

    while(isADCBusy(ADC_IR)); //Wait until IR ADC isn't busy
    while(isADCBusy(ADC_CURRENT_SENSE)); //Wait until Current Sense ADC isn't busy

    char* value = new char();

    //Set up config, shudown mode, interupts disabled and cleared
    *value = 0x08;
    writetoreg_i2c(ADC_IR,            ADC_CONFIG, 1, value);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CONFIG, 1, value);

    //Turn off all interupts
    *value = 0xFF;
    writetoreg_i2c(ADC_IR,            ADC_INT_MASK, 1, value);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_INT_MASK, 1, value);

    //Set conversion rate to continous (WARNING: most be done in shutdown mode)
    *value = 0x01;
    writetoreg_i2c(ADC_IR,            ADC_CONVERSION_RATE, 1, value);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CONVERSION_RATE, 1, value);

    //Set channel disables (WARNING: most be done in shutdown mode)
    *value = 0;
    writetoreg_i2c(ADC_IR,            ADC_CHANNEL_DISABLE, 1, value);        //Turn on all IR's
    *value = 0xFC;
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CHANNEL_DISABLE, 1, value);        //Turn on ch0 and 1

    //Set the advance config, use external board reference of 4.096 V ref, mode 0
    *value = 0;
    writetoreg_i2c(ADC_IR,            ADC_ADV_CONFIG, 1, value);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_ADV_CONFIG, 1, value);

    //Turn on conversion
    *value = 0x09;
    writetoreg_i2c(ADC_IR,            ADC_CONFIG, 1, value);
    writetoreg_i2c(ADC_CURRENT_SENSE, ADC_CONFIG, 1, value);

    //clean up
    delete value;

    return true;
}