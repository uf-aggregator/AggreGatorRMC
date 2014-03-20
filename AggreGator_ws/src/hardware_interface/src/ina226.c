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


void main(void) {
    int file;
    char filename[40];
    char reg_ptr_addr[1] = {0x02}; //bus voltage register address
    const char *buffer;
    int addr = 0x40;        // The I2C address of the ADC
    
    char config_reg[3] = {0}; //address is 0
    config_reg[1] = 0x05; //high byte 0b00001111
    config_reg[0] =  0x27; //low byte 0b11100111;

//open the file to interface with the i2c bus
    sprintf(filename,"/dev/i2c-1");
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

//see if there's a device at "addr"
    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    char buf[10] = {0};
    float data;
    char channel;

    if(write(file, config_reg, 3) != 3) {
    
	//ERROR HANDLING: i2c transaction failed
	printf("Failed to write to the i2c bus.\n\n");
    }     

    if(write(file,reg_ptr_addr,1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n\n");
   //     buffer = g_strerror(errno);
//        printf(buffer);
      printf("\n\n");
    }
    

    int i = 0;
    while(1)//for(i = 0; i<4; i++) {
      {  // Using I2C Read
        if (read(file,buf,2) != 2) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
     //       buffer = g_strerror(errno);
    //        printf(buffer);
            printf("\n\n");
        } else {
            data = (float)(buf[0]<<8)+buf[1];
            data = data*1.25/1000;
            //channel = ((buf[0] & 0b00110000)>>4);
            system("clear");
	    printf("Data:  %04f V\n",data);
	    usleep(500000);	
	 }
    }   
}

