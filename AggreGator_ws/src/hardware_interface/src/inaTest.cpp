#include "i2c.h"
#include <stdio>
#include <iostream>

int main(void)
{
	init_i2c();
	
	writetoreg_i2c(0x40,'0',1,0x4F27); //set configuration register
	
	writetoreg_i2c(0x40,'5',1,84); //set calibration register
	
	while(true)
	{
		cout << readfromreg_i2c(0x40,'1',1) << endl;
		
	return 0;
]
