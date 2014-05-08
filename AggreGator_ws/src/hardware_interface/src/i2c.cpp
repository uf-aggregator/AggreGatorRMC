/*This file contains fucntion definitions that interface to the O-Droid's I2C headers*/

#include "i2c.h"

char* read_i2c(int address, int size) 
{
	int file; //the "file" that will be used to interface with the i2c bus
        char* read_buffer = (char*) malloc(MAX_READ_BYTES);
	read_buffer[0] = 1; //1 means data is not valid
	
    if (size > MAX_READ_BYTES)
	{
		printf("Cannot read that many bytes!");
		return read_buffer;
	}
	
    char filename[40] = FILENAME;
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
	close(file);
        return read_buffer;
    }
	
	if (read(file,read_buffer+1,size) != size) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
            /*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
            printf("\n\n");
			read_buffer[0] = 1; //1 means data is not valid
			close(file);
			return read_buffer;
        } else {
			//for(int i = size; i >0; i--) //shift all data right one bit
			//	read_buffer[i] = read_buffer[i-1];
			read_buffer[0] = 0;//0 for valid data, 1 else
		close(file);
            return read_buffer;
	 }
	close(file);
     return read_buffer;
}	

char* readfromreg_i2c(int address, char reg_ptr_addr, int size)
{
	int file; //the "file" that will be used to interface with the i2c bus
        char* read_buffer = (char*) malloc(MAX_READ_BYTES);
	read_buffer[0] = 1; //1 means data is not valid
	
    if (size > MAX_READ_BYTES)
	{
		printf("Cannot read that many bytes!");
		return read_buffer;
	}
	
    char filename[40] = FILENAME;
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
	close(file);
        return read_buffer;
    }
	//set the register to be read from
    if(write(file,&reg_ptr_addr,1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n\n");
   //     error_buffer = g_strerror(errno);
        printf("Errno: %i\n", errno);
		close(file);
		return read_buffer;
    }
	
	if (read(file,read_buffer+1,size) != size) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
            /*error_buffer = g_strerror(errno);
            printf(error_buffer);*/
            printf("\n\n");
			read_buffer[0] = 1; //1 means data is not valid
			close(file);
			return read_buffer;
        } else {
			//for(int i = size; i >0; i--) //shift all data right one bit
			//	read_buffer[i] = read_buffer[i-1];
			read_buffer[0] = 0;//0 for valid data, 1 else
		close(file);
            return read_buffer;
	 }
	 close(file);
    return read_buffer;
}	

int writetoreg_i2c(int address, char reg_ptr_addr, int size, char* data)
{
	int file; //the "file" that will be used to interface with the i2c bus
    if (size > MAX_WRITE_BYTES)
	{
		printf("Cannot write that many bytes!");
		return 1;
	}
	
    char write_buffer[MAX_WRITE_BYTES];
    char filename[40] = FILENAME;
	if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
		//error_buffer = g_strerror(errno);
            printf("Error #%i\n",errno);
        return 1;
    }
	
	if (ioctl(file,I2C_SLAVE,address) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
	//	error_buffer = g_strerror(errno);
            printf("Error #%i",errno);

	close(file);
        return 1;
    }
	
	write_buffer[0] = reg_ptr_addr;
	
        int i = 1;
	for(i = 1; i <= size; i++) //copy data into write_buffer
		write_buffer[i]=data[i-1];
	
    if(write(file, write_buffer, size+1) != size+1) {
		//ERROR HANDLING: i2c transaction failed
           	printf("Error number: %i\n",errno);
		close(file);
		return 1;
    }  
	close(file);
	return 0;
}

int write_i2c(int address, int size, char* data)
{
	int file; //the "file" that will be used to interface with the i2c bus
    if (size > MAX_WRITE_BYTES)
	{
		printf("Cannot write that many bytes!");
		return 1;
	}
	
    char write_buffer[MAX_WRITE_BYTES];

    char filename[40] = FILENAME;
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
	close(file);
        return 1;
    }
	
    if(write(file, data, size) != size) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to write to the i2c bus.\n\n");
		printf("Error: %i", errno);
		close(file);
		return 1;
    }  
	close(file);
	return 0;
}

//Initializes I2C by setting things to their default states and setting the logic level from the O-Droid
int init_i2c(void)
{
	//Turn on lv output pin for all logic level converters
	setGPIOWrite(17,1);
	return 0;
}

//Performs a software reset on all devices on the I2C bus
int softwareReset(void)
{
	char rst[] = {0x6};
	write_i2c(0x0,1,rst); //Software reset command
	
	return 0;
}
