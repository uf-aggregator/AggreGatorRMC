#include "ina226.h"

//Sets configuration and calibration registers
void INA226::inaInitialize()
{
	//Set cconfiguration register
	common_files::WriteI2CRegister configure;
	configure.addr = ina226Addr;
	configure.reg = config;
	configure.data.push_back(0x4B); //Set averaging mode and conversion times on INA226
	configure.data.push_back(0x27);
	write_reg_pub.publish(configure); //Publish message containing information to write to the configuration register
	
	//Set Calibration register
	common_files::WriteI2CRegister calibration;
	calibration.addr = ina226Addr;
	calibration.reg = calibrate;
	
	int cal = 0.00512/(currentLSB*RSHUNT); //value to load into the calibration register. Determined by the shunt resistor and the max expected current that the INA226 will be sensing.

	calibration.data.push_back(cal>>8); //Sets calibration used in calculating current, and power within the INA226 chip
	calibration.data.push_back(cal);
	write_reg_pub.publish(calibration); //Publish message containing information to writeo to the calibration register
}

//Reads power from the INA226 by requesting a service call to the i2c_node
int INA226::readElectronicPower()
{
	common_files::ReadI2CRegister readPower;
	
	readPower.request.addr = ina226Addr; //Sets request address
	readPower.request.size = 2; //Specifies size of return data
	
	readPower.request.reg = power; //Specifies specific register to read from in the INA226. We want to read from the power register
	
	if(!read_register_srv.call(readPower))
	{
		ROS_ERROR("Error reading from the INA226 power register!");
		return 0;
	}
	else
		return (int)(readPower.response.data[0]<<8) + readPower.response.data[1];
}

//Publishes raw power data gathered from the readElectronicPower function
void INA226::publishElectronicPower()
{
	common_files::ElectronicPowerData sendPowerData;
	//ROS_INFO("POWER: %i",readElectronicPower());
	sendPowerData.power = readElectronicPower()*powerLSB; //grab power reading from service, multiply it by the powerLSB conversion ratio to get Watts, then publish to power monitoring node
	
	power_pub.publish(sendPowerData); //publish
}