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

#define max_read_bytes 41;
#define max_write_bytes 40;
#define filename "/dev/i2c-1";
//reads "size" bytes from the device at "address". 
char* read_i2c(int address, int size); 

//reads "size" bytes from the register with address "reg_ptr_address" on the device at "address". Returns a pointer, with the first byte the dirty flag(1 for dirty, 0 for clean). The data will begin from the next byte. 
char* readfromreg_i2c(int address, char reg_ptr_addr, int size);

//writes "size" bytes (pointed to by "data") to the device at "address". Returns 1 on failure, 0 on success
int write_i2c(int address, int size, char* data);

//writes "size" bytes (pointed to by "data") to the register at "reg_ptr_address" on the device at "address"
int writetoreg_i2c(int address, char reg_ptr_addr, int size, char* data);

//currently not defined, for future use
int init_i2c(void);


