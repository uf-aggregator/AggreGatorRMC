#ifndef _I2C_H
#define _I2C_H


#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "GPIO.h"
#include "ros/ros.h"
#include "common_msgs/WriteI2C.h"
#include "hardware_interface/ReadI2C.h"
#include "common_msgs/WriteI2CRegister.h"
#include "hardware_interface/ReadI2CRegister.h"

#define MAX_READ_BYTES 41
#define MAX_WRITE_BYTES 40
#define FILENAME "/dev/i2c-1"

class I2C {
	public:
	I2C(){}
	~I2C(){}

	//reads "size" bytes from the device at "address".
	//User must free memory when complete
	static char* read_i2c(int address, int size); 

	//reads "size" bytes from the register with address "reg_ptr_address" on the device at "address".
	//Returns a pointer, with the first byte the dirty flag(1 for dirty, 0 for clean). The data will begin from the next byte.
	//User must free memory when complete
	static char* readfromreg_i2c(int address, char reg_ptr_addr, int size);

	//writes "size" bytes (pointed to by "data") to the device at "address". Returns 1 on failure, 0 on success
	static int write_i2c(int address, int size, char* data);

	//writes "size" bytes (pointed to by "data") to the register at "reg_ptr_address" on the device at "address"
	static int writetoreg_i2c(int address, char reg_ptr_addr, int size, char* data);

	//Init I2C, returns 0 if completed correctly
	static int init_i2c(void);

	//Performs a software reset on the I2C bus
	static int softwareReset(void);

	//callbacks
	static bool ReadI2CCallback(hardware_interface::ReadI2C::Request&  request,
                     hardware_interface::ReadI2C::Response& reply);

	static bool ReadRegisterI2CCallback(hardware_interface::ReadI2CRegister::Request&  request,
                             hardware_interface::ReadI2CRegister::Response& reply);

	static void WriteI2CCallback(const common_msgs::WriteI2C& msg);

	static void WriteRegisterI2C(const common_msgs::WriteI2CRegister& msg);
};

#endif
