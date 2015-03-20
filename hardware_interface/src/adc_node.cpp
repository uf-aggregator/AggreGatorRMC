/*
 * ADC Node
 *
 * Description:  Handels the initilization and use of the ADC's
 *
 * Subscribes to:
 *
 * Publishes to:
 *      /ir_raw
 *      /power_monitoring
 *
 * Services used:
 *
 *
 * Notes:
 *
 *      Writes are composed of ADC address, register address, data to write
 *
 *      Reads:
 *              First if not done previously write the ADC address then immediately write the address of the register
 *              Then read from the ADC as you would normally (write the address out)
 *
 *      Digital output = [(Inx - GND) / VREF] * 2^12
 *
 *      External VREF is 4.096
 */

#include "ros/ros.h"
#include "std_msgs/builtin_uint8.h"

#include "common_files/WriteI2CRegister.h"
#include "common_files/ReadI2CRegister.h"
#include "common_files/RawIRData.h"
#include "common_files/RawMotorPowerData.h"


//Global varibles
ros::Publisher ir_pub;                  //Publishes ir info
ros::Publisher motor_power_pub;         //Publishes power usage info
ros::Publisher write_register_pub;      //Writes to an I2C Register

ros::ServiceClient read_register_svr;   //Reads I2C registers


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
 * Check the busy flag
 *
 * Poll for the busy flag returns true if busy, false if not
 */
bool isADCBusy(ADC adc)
{
    //Define and fill in a service request
    common_files::ReadI2CRegister read;
    read.request.addr = adc;
    read.request.reg = ADC_BUSY;
    read.request.size = 1;

    //Send request
    if(read_register_svr.call(read))
    {
        //Sussecful read
        if(read.response.data[0])
        {
            //Still busy
            return true;
        }
        else
        {
            //Not busy!
            return false;
        }
    }
    else
    {
        //Unsussesful read!!!
        //Do error handeling
    }

    return false;
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

    //Define a read service
    common_files::ReadI2CRegister read;

    //Read 2 bytes from register reg on ADC adc
    read.request.addr = adc;
    read.request.reg = reg;
    read.request.size = 2;
    if(!read_register_svr.call(read))
    {
        //Error handleing
        ROS_WARN("Failed to read from register %i at address %i", reg, adc);
        return 0;
    }

    //convert to a unsigned 16 bit integer
    return ((((u_int16_t) read.response.data[1]) << 8) + (u_int16_t) (read.response.data[0]));
}

/*
 * Initilise all adcs
 *
 * Returns true if succsful false if unsuccesfull
 * This function makes the assumption that 33ms have elapsed since the ADC was powered up
 */
bool InitADC()
{
    //Wait for ADC's to be ready
    while(isADCBusy(ADC_IR));               //Wait until IR ADC isn't busy
    while(isADCBusy(ADC_CURRENT_SENSE));    //Wait until Current Sense ADC isn't busy

    //Define a msg for repeted use
    common_files::WriteI2CRegister write;

    //Set up config, shudown mode, interupts disabled and cleared (0x08)
    write.addr      = ADC_IR;
    write.reg       = ADC_CONFIG;
    write.data.push_back(0x08);
    write_register_pub.publish(write);
    write.addr      = ADC_CURRENT_SENSE;
    write_register_pub.publish(write);

    //Turn off all interupts (0xFF)
    write.addr      = ADC_IR;
    write.reg       = ADC_INT_MASK;
    write.data[0]   = 0xFF;
    write_register_pub.publish(write);
    write.addr      = ADC_CURRENT_SENSE;
    write_register_pub.publish(write);

    //Set conversion rate to continous (0x01) (WARNING: most be done in shutdown mode)
    write.addr      = ADC_IR;
    write.reg       = ADC_CONVERSION_RATE;
    write.data[0]  = 0x01;
    write_register_pub.publish(write);
    write.addr      = ADC_CURRENT_SENSE;
    write_register_pub.publish(write);

    //Set channel disables (0x00, 0xFC) (WARNING: most be done in shutdown mode)
    write.addr      = ADC_IR;
    write.reg       = ADC_CHANNEL_DISABLE;
    write.data[0]  = 0x00;
    write_register_pub.publish(write);
    write.addr      = ADC_CURRENT_SENSE;
    write.data[0]   = 0xFC;
    write_register_pub.publish(write);

    //Set the advance config, use external board reference of 4.096 V ref, mode 0 (0x01)
    write.addr      = ADC_IR;
    write.reg       = ADC_ADV_CONFIG;
    write.data[0]   = 0x01;
    write_register_pub.publish(write);
    write.addr      = ADC_CURRENT_SENSE;
    write_register_pub.publish(write);

    //Turn on conversion (0x09)
    write.addr      = ADC_IR;
    write.reg       = ADC_CONFIG;
    write.data[0]   = 0x09;
    write_register_pub.publish(write);
    write.addr      = ADC_CURRENT_SENSE;
    write_register_pub.publish(write);

    return true;
}

/*
 * Read and publish raw ir_data
 */
void PublishIRData()
{
    //Generate a msg
    common_files::RawIRData ir_data;

    //Loop through all IR's
    for(int iii = ADC_CH0, jjj = 0; iii <= ADC_CH7; ++iii, ++jjj)
    {
        ir_data.data[jjj] = ReadADC(ADC_IR, static_cast<ADC_REG>(iii));
    }

    //Publish the msg
    ir_pub.publish(ir_data);
}

/*
 * Read and publish voltage and current for power calculations
 */
void PublishMotorPowerData()
{
    //Generate the msg
    common_files::RawMotorPowerData power_data;

    //Read the data
    power_data.current = ReadADC(ADC_CURRENT_SENSE, ADC_CH0);
    power_data.voltage = ReadADC(ADC_CURRENT_SENSE, ADC_CH1);

    //Publish the msg
    motor_power_pub.publish(power_data);
}

/*
 * Main loop
 */
int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the adc node
    ros::init(argc, argv, "adc_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize publishers, subscribers, and services
    write_register_pub = n.advertise<common_files::WriteI2CRegister>("write_i2c_register", 1000);
    motor_power_pub = n.advertise<common_files::RawMotorPowerData>("raw_motor_power", 1000);
    ir_pub = n.advertise<common_files::RawIRData>("raw_ir", 1000);

    read_register_svr = n.serviceClient<common_files::ReadI2CRegister>("read_i2c_register");

    //Initilize the ADCs
    ROS_INFO("Initializing the ADC");
    if(InitADC())
    {
        ROS_INFO("Initialzed the ADC");
    }
    else
    {
        ROS_ERROR("Failed to initialize the ADC");
    }

    /*
     * Main loop
     */
    while (ros::ok())
    {
        PublishIRData();
        PublishMotorPowerData();
        ros::spinOnce();
    }
}
