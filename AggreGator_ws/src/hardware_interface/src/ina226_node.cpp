#include "ros/ros.h"

#include "hardware_interface/WriteI2CRegister.h"
#include "hardware_interface/ReadI2CRegister.h"
#include "hardware_interface/ElectronicPowerData.h"

#define		MAX_EXPECTED_CURRENT	2 //Max expected currrent flowing through the shunt, in amps
#define		RSHUNT					1 //Shunt register value, in ohms

float currentLSB = MAX_EXPECTED_CURRENT/(2^15); //Current LSB to be used to find Power LSB which will be paired with the power raw data to be processed in power monitoring node. Units in Amps/bit
float powerLSB = 25*currentLSB; //Power lsb to convert raw power data to a usable value. Units in Watts/bit

ros::Publisher write_reg_pub; //publisher used to send data to INA226 on the I2C bus
ros::Publisher power_pub; //publishes to the power monitoring node for processing

ros::ServiceClient read_register_srv; //Reads registers from INA226

//All important register addresses within the INA226 chip
enum regAddr
{
	ina226Addr			=0x40,
	config				= 0x0,
	shuntVoltage,
	busVoltage,
	power,
	current,
	calibrate,
	alertControl,
	alertLimit
};

//Sets configuration and calibration registers
void inaInitialize()
{
	//Set cconfiguration register
	hardware_interface::WriteI2CRegister configure;
	configure.addr = ina226Addr;
	configure.reg = config;
	configure.data.push_back(0x4B27); //Set averaging mode and conversion times on INA226
	
	write_reg_pub.publish(configure); //Publish message containing information to write to the configuration register
	
	//Set Calibration register
	hardware_interface::WriteI2CRegister calibration;
	calibration.addr = ina226Addr;
	calibration.reg = calibrate;
	
	int cal = 0.00512/(currentLSB*RSHUNT); //value to load into the calibration register. Determined by the shunt resistor and the max expected current that the INA226 will be sensing.
	
	calibration.data.push_back(cal); //Sets calibration used in calculating current, and power within the INA226 chip
	
	write_reg_pub.publish(calibration); //Publish message containing information to writeo to the calibration register
}

//Reads power from the INA226 by requesting a service call to the i2c_node
int readElectronicPower()
{
	hardware_interface::ReadI2CRegister readPower;
	
	readPower.request.addr = ina226Addr; //Sets request address
	readPower.request.size = 2; //Specifies size of return data
	
	readPower.request.reg = power; //Specifies specific register to read from in the INA226. We want to read from the power register
	
	if(!read_register_srv.call(readPower))
		ROS_ERROR("Error reading from the INA226 power registr!");
	
	return readPower.response.data[0]<<8+readPower.response.data[1];
}

//Publishes raw power data gathered from the readElectronicPower function
void publishElectronicPower()
{
	hardware_interface::ElectronicPowerData sendPowerData;
	
	sendPowerData.power = readElectronicPower(); //grab values from serice response, put them into 2 byres, and store into the message to be sent to power monitoring node for processing
	sendPowerData.powerLSB = powerLSB;
	
	power_pub.publish(sendPowerData); //publish

}

int main(int argc, char** argv)
{
 
    //Initilize the ina226 node
    ros::init(argc, argv, "ina226_node");

    //Node handler this is how you work with ROS

    ros::NodeHandle n;
	
	//Set up publishers
	write_reg_pub = n.advertise<hardware_interface::WriteI2CRegister>("write_i2c_register",1000);
	power_pub = n.advertise<hardware_interface::ElectronicPowerData>("electronic_power",1000);
	
	//Set up service client
	read_register_srv = n.serviceClient<hardware_interface::ReadI2CRegister>("read_i2c_register");
	
	while(write_reg_pub.getNumSubscribers()==0); //Wait until the publisher is fully connected to the subscriber
	
	//Initialize the INA226 chip
	inaInitialize();
	
    /*
     * Main loop
     */
    while (ros::ok())
    {
	publishElectronicPower();
        ros::spinOnce();
    }
	
	return 0;
}
