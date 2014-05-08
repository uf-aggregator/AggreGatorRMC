
#include "i2c.h"
#include <iostream>

int main(void)
{
	softwareReset();
	init_i2c();

	char config[] = {0x4F,0x27};

	writetoreg_i2c(0x40,'0',2,config); //set configuration register


	char calibrate[] = {0x00,0x54};
	writetoreg_i2c(0x40,'5',2,calibrate); //set calibration register
	
	char regPointer[] = {0};
	char* read_buffer;
	float data;
	while(true)
	{	
		regPointer[0]= '2';
		write_i2c(0x40,1,regPointer);
		read_buffer = read_i2c(0x40,2);
		data = (float)(read_buffer[1]<<8)+read_buffer[2];
		data = data*.00125;
		std::cout << "Bus Voltage: " << data  << std::endl;
		regPointer[0] = '1';
		write_i2c(0x40,1,regPointer);
		read_buffer = read_i2c(0x40,2);
		data = (float)(read_buffer[1]<<8)+read_buffer[2];
		data = data*0.0000025;
		std::cout << "Shunt Voltage: " << data  << std::endl;
		regPointer[0] = '4';
		write_i2c(0x40,1,regPointer);
		read_buffer = read_i2c(0x40,2);
		data = (float)(read_buffer[1]<<8)+read_buffer[2];
		data = data*0.000061;
		std::cout << "Current Sensed: " << data  << std::endl;
		regPointer[0] = '3';
		write_i2c(0x40,1,regPointer);
		read_buffer = read_i2c(0x40,2);
		data  = (float)(read_buffer[1]<<8)+read_buffer[2];
		data = data*0.001525;
		std::cout << "Power: " << data  << std::endl;
	}
	return 0;
}

