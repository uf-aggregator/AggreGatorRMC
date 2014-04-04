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

#define MAX_READ_BYTES 41
#define MAX_WRITE_BYTES 40
#define FILENAME "/dev/i2c-1"

//reads "size" bytes from the device at "address".
//User must free memory when complete
char* read_i2c(int address, int size); 

//reads "size" bytes from the register with address "reg_ptr_address" on the device at "address".
//Returns a pointer, with the first byte the dirty flag(1 for dirty, 0 for clean). The data will begin from the next byte.
//User must free memory when complete
char* readfromreg_i2c(int address, char reg_ptr_addr, int size);

//writes "size" bytes (pointed to by "data") to the device at "address". Returns 1 on failure, 0 on success
int write_i2c(int address, int size, char* data);

//writes "size" bytes (pointed to by "data") to the register at "reg_ptr_address" on the device at "address"
int writetoreg_i2c(int address, char reg_ptr_addr, int size, char* data);



//Init I2C, returns 0 if completed correctly
int init_i2c(void);

//Performs a software reset on the I2C bus
int softwareReset(void);

#endif //_I2C_H
