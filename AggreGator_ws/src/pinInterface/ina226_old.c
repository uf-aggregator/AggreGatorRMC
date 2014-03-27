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

#define max_read_bytes 40;
#define max_write_bytes 40;
#define filename "/dev/i2c-1";
//make init, read, write function1s
int init_i2c(void);
char* read_i2c(int address, int size)
{
	int file; //the "file" that will be used to interface with the i2c bus
	char read_buffer[max_read_bytes];
	read_buffer[0] = 1; //0 means data is not valid
	
	if (size > max_read_bytes)
	{
		printf("Cannot read that many bytes!");
		return read_buffer;
	}
	
	char filename[40] = "/dev/i2c-1";
	if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return read_buffer;
    }
	
	if (ioctl(file,I2C_SLAVE,address) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return read_buffer;
    }
	
	if (read(file,read_buffer,size) != size) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
            /*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
            printf("\n\n");
			read_buffer[0] = 1; //0 means data is not valid
			return read_buffer;
        } else {
			for(int i = size; i >0; i--) //shift all data right one bit
				read_buffer[i] = read_buffer[i-1];
			read_buffer[0] = 0;//1 for valid data, 0 else
            return read_buffer;
	 }
}	

char* readfromreg_i2c(int address, char reg_ptr_addr, int size)
{
	int file; //the "file" that will be used to interface with the i2c bus
	char read_buffer[max_read_bytes];
	read_buffer[0] = 1; //0 means data is not valid
	
	if (size > max_read_bytes)
	{
		printf("Cannot read that many bytes!");
		return read_buffer;
	}
	
	char filename[40] = "/dev/i2c-1";
	if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return read_buffer;
    }
	
	if (ioctl(file,I2C_SLAVE,address) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return read_buffer;
    }
	//set the register to be read from
    if(write(file,reg_ptr_addr,1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n\n");
   //     error_buffer = g_strerror(errno);
//        printf(error_buffer);
      printf("\n\n");
    }
	
	if (read(file,read_buffer,size) != size) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
            /*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
            printf("\n\n");
			read_buffer[0] = 1; //0 means data is not valid
			return read_buffer;
        } else {
			for(int i = size; i >0; i--) //shift all data right one bit
				read_buffer[i] = read_buffer[i-1];
			read_buffer[0] = 0;//1 for valid data, 0 else
            return read_buffer;
	 }
}	

int writetoreg_i2c(int address, char reg_ptr_addr, int size, char* data)
{
	int file; //the "file" that will be used to interface with the i2c bus
	char write_buffer[max_read_bytes];
	if (size > max_write_bytes)
	{
		printf("Cannot write that many bytes!");
		return 1;
	}
	
	char filename[40] = "/dev/i2c-1";
	if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return 1;
    }
	
	if (ioctl(file,I2C_SLAVE,address) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return 1;
    }
	
	write_buffer[0] = reg_ptr_addr;
	
	for(int i = 1; i <= size; i++)
		write_buffer[i]=data[i-1];
	
    if(write(file, write_buffer, size+1) != size+1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to write to the i2c bus.\n\n");
		return 1;
    }  
	
	return 0;
}

int write_i2c(int address, int size, char* data)
{
	int file; //the "file" that will be used to interface with the i2c bus
	char write_buffer[max_read_bytes];
	if (size > max_write_bytes)
	{
		printf("Cannot write that many bytes!");
		return 1;
	}
	
	char filename[40] = "/dev/i2c-1";
	if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return 1;
    }
	
	if (ioctl(file,I2C_SLAVE,address) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		/*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
        return 1;
    }
	

    if(write(file, data, size) != size) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to write to the i2c bus.\n\n");
		return 1;
    }  
	
	return 0;
}


void main(void) {
    int file; //the "file" that will be used to interface with the i2c bus
    char filename[40]; //name of said file
    
    const char *error_buffer; //buffer to store error messages
	char read_buffer[2] = {0}; //buffer to store data from device 
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

//open the file to interface with the i2c bus
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

//see if there's a device at "addr" --this needs to be done anytime writing to a new address, as all subsequent reads/writes will use the address used here
    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

//write desired settings to configuration register
    if(write(file, config_reg, 3) != 3) {
    //ERROR HANDLING: i2c transaction failed
	printf("Failed to write to the i2c bus.\n\n");
    }     

//set the register to be read from
    if(write(file,reg_ptr_addr,1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n\n");
   //     error_buffer = g_strerror(errno);
//        printf(error_buffer);
      printf("\n\n");
    }
    
//continuously read   
    while(1)
      {  // Using I2C Read
        if (read(file,read_buffer,2) != 2) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
     //       error_buffer = g_strerror(errno);
    //        printf(error_buffer);
            printf("\n\n");
        } else {
            data = (float)(read_buffer[0]<<8)+read_buffer[1];
            data = data*1.25/1000;
            system("clear");
	    printf("Data:  %04f V\n",data);
	    usleep(500000);	
	 }
    }   
}

