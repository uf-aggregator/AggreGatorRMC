#ifndef INA_226
#define INA_226

#include "ros/ros.h"
#include "common_msgs/WriteI2CRegister.h"
#include "hardware_interface/ReadI2CRegister.h"
#include "common_msgs/ElectronicPowerData.h"

#define	MAX_EXPECTED_CURRENT 	 2//Max expected currrent flowing through the shunt, in amps
#define	RSHUNT			 0.004 //Shunt register value, in ohms

float currentLSB = MAX_EXPECTED_CURRENT/(pow(2,15)); //Current LSB to be used to find Power LSB which will be paired with the power raw data to be processed in power monitoring node. Units in Amps/bit
float powerLSB = 25*currentLSB; //Power lsb to convert raw power data to a usable value. Units in Watts/bit

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
	ros::Publisher write_reg_pub; //publisher used to send data to INA226 on the I2C bus
	ros::Publisher power_pub; //publishes to the power monitoring node for processing
	ros::ServiceClient read_register_srv; //Reads registers from INA226

class INA226 {
	public:
		static void inaInitialize();
		static int readElectronicPower();
		static void publishElectronicPower();
};

#endif