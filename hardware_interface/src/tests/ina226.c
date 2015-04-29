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
#include "hardware_interface/i2c.h"

int main(void) {
    int file; //the "file" that will be used to interface with the i2c bus
    char filename[40]; //name of said file
    int i2c_status = 0; 
    const char *error_buffer; //buffer to store error messages
	char read_buffer[3] = {0}; //buffer to store flag and data from device 
    float data; //to be used to store data once read	
	int addr = 0x40;        // The I2C address of the ADC
    char reg_ptr_addr[1] = {0x02}; //bus voltage register address
    char config_reg[3] = {0}; //address of configuration register is 0x00
//set configuration values -- see datasheet for what each bit means
//DEFAULT: 0b0100000100100111
//15- reset, equivalent to power on
//14:12- reserved
//11:9- number of averages
//8:6- conversion time
//5:3- shunt voltage conversion time
//2:0- operating mode

//1024 averages, 1.1ms conversion time, continuous bus voltage
	config_reg[1] = 0b01001111; //high byte 
    config_reg[2] = 0b00100110; //low byte 

//write the name of the file that needs to be opened to interface with the i2c bus	
    sprintf(filename,"/dev/i2c-1");

//write configuration settings
	i2c_status = writetoreg_i2c(addr, config_reg[0], 2, config_reg+1);
	if(i2c_status == 1)
		return 1;
		
//set the register to be read from
    i2c_status = write_i2c(addr, 1, &reg_ptr_addr);
    if(i2c_status == 1)
		return 1;
		
//continuously read   
    while(1)
	{  
        read_buffer = read_i2c(addr, 2); 
		if(read_buffer[0]==1)
			return 1;
		else
		{
            data = (float)(read_buffer[1]<<8)+read_buffer[2];
            data = data*1.25/1000;
            system("clear");
	    printf("Data:  %04f V\n",data);
	    usleep(500000);	
		}
    }   
}

