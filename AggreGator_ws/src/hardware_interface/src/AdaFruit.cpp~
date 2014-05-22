/*Daniel Kelly
UF AggreGator
	This contains definitions for the necessary interfacing functions to the Adafruit PWM generator, which will help control the motors
	*/
#include "AdaFruit.h"

using namespace std;

//Initializes the AdaFruit PWM generator by setting the MODE1 register accordingly.
pwmRegisterData AdaFruitInit()
{
	pwmRegisterData initialize;
	init_i2c(); //Initializes the I2C bus

	int data[] = {0x0,0b00000001}; //Leave everything in MODE1 as default, except turn off SLEEP bit
	initialize.initData = data;
	return initialize;
	
	//write_i2c(AdaFruitAddr,2,data); //Writes above data to the Adafruit

}


//Sets the PWM for the left front motor (assigned to PWM channel 3)
//Input is a percentage value
pwmRegisterData setLeftFrontMotor(float PWM)
{

	int on_count = PWM * .01 * 4095; //Converts input to decimal and gets correct counter value
	int on_count_high = (on_count/256); //Isolates higher 4 bits of count value
	int on_count_low = (on_count & 255);//Isolates lower 8 bits of count value

	pwmRegisterData leftFrontMotor;
	//Value for lower 8 bit ON register
	int lowRegOn[] = {leftFrontMotorChannel_ON_LOW,0}; //ON values are set to 0 so the counter turns 		ON at count value of 0, or at the beginning of the PWM cycle
	leftFrontMotor.lowRegOn = lowRegOn;
	//Writes value to Adafruit
	//write_i2c(AdaFruitAddr,2,lowRegOn);

	//Value for higher 8 bit register
	int highRegOn[] = {leftFrontMotorChannel_ON_HIGH,0};
	leftFrontMotor.highRegOn = highRegOn;
	//write_i2c(AdaFruitAddr,2,highRegOn);

	//Value for lower 8 bit OFF register
	int lowRegOff[] = {leftFrontMotorChannel_OFF_LOW,on_count_low};//Sets the count values for the PWM to turn OFF
	leftFrontMotor.lowRegOff = lowRegOff;
	//write_i2c(AdaFruitAddr,2,lowRegOff);

	//Value for higher 8 bit OFF register
	int highRegOff[] = {leftFrontMotorChannel_OFF_HIGH,on_count_high};
	leftFrontMotor.highRegOff = highRegOff;
	//write_i2c(AdaFruitAddr,2,highRegOff);
	return leftFrontMotor;
}

//Sets the PWM for the left front motor (assigned to PWM channel 0)
//Input is a percentage value
pwmRegisterData setLeftRearMotor(float PWM)
{

	int on_count= PWM * .01 *4095; //Converts input to decimal and gets correct counter value
	int on_count_high = (on_count/256); //Isolates higher 4 bits of count value
	int on_count_low = (on_count & 255); //Isolates lower 8 bits of count value

	pwmRegisterData leftRearMotor;
	//Value for lower 8 bit ON register
	int lowRegOn[] = {leftRearMotorChannel_ON_LOW,0};//ON values are set to 0 so the counter turns 		ON at count value of 0, or at the beginning of the PWM cycle
	leftRearMotor.lowRegOn = lowRegOn;
	//write_i2c(AdaFruitAddr,2,lowRegOn);
	//Value for higher 8 bit register
	int highRegOn[] = {leftRearMotorChannel_ON_HIGH,0};
	leftRearMotor.highRegOn = highRegOn;
	//write_i2c(AdaFruitAddr,2,highRegOn);
	//Value for lower 8 bit OFF register
	int lowRegOff[] = {leftRearMotorChannel_OFF_LOW,on_count_low};//Sets the count values for the PWM to turn OFF
	leftRearMotor.lowRegOff = lowRegOff;
	//write_i2c(AdaFruitAddr,2,lowRegOff);
	//Value for higher 8 bit OFF register
	int highRegOff[] = {leftRearMotorChannel_OFF_HIGH,on_count_high};
	leftRearMotor.highRegOff = highRegOff;
	//write_i2c(AdaFruitAddr,2,highRegOff);	
	return leftRearMotor;
}

//Sets the PWM for the left front motor (assigned to PWM channel 4)
//Input is a percentage value
pwmRegisterData setRightRearMotor(float PWM)
{

	int on_count = PWM * .01 * 4095;//Converts input to decimal and gets correct counter value
	int on_count_high = (on_count/16); //Isolates higher 4 bits of count value
	int on_count_low = (on_count & 255);//Isolates lower 8 bits of count value
	//Value for lower 8 bit ON register
	
	pwmRegisterData rightRearMotor;
	
	int lowRegOn[] = {rightRearMotorChannel_ON_LOW,0};//ON values are set to 0 so the counter turns 		ON at count value of 0, or at the beginning of the PWM cycle
	rightRearMotor.lowRegOn = lowRegOn;
	//write_i2c(AdaFruitAddr,2,lowRegOn);
	//Value for higher 8 bit register
	
	int highRegOn[] = {rightRearMotorChannel_ON_HIGH,0};
	rightRearMotor.highRegOn = highRegOn;
	
	//write_i2c(AdaFruitAddr,2,highRegOn);
	//Value for lower 8 bit OFF register
	int lowRegOff[] = {rightRearMotorChannel_OFF_LOW,on_count_low};
	rightRearMotor.lowRegOff = lowRegOff;
	//write_i2c(AdaFruitAddr,2,lowRegOff);
	//Value for higher 8 bit OFF register
	int highRegOff[] = {rightRearMotorChannel_OFF_HIGH,on_count_high};
	rightRearMotor.highRegOff = highRegOff;
	
	//write_i2c(AdaFruitAddr,2,highRegOff);

	return rightRearMotor;
}

//Sets the PWM for the left front motor (assigned to PWM channel 1)
//Input is a percentage value
pwmRegisterData setRightFrontMotor(float PWM)
{
	int on_count = PWM * .01 * 4095;//Converts input to decimal and gets correct counter value
	int on_count_high = (on_count/256);//Isolates higher 4 bits of count value
	int on_count_low = (on_count & 255);//Isolates lower 8 bits of count value
	//Value for lower 8 bit ON register
	
	pwmRegisterData rightFrontMotor;
	
	int lowRegOn[] = {rightFrontMotorChannel_ON_LOW,0};//ON values are set to 0 so the counter turns 		ON at count value of 0, or at the beginning of the PWM cycle
	rightFrontMotor.lowRegOn = lowRegOn;
	//write_i2c(AdaFruitAddr,2,lowRegOn);
	//Value for higher 8 bit register
	int highRegOn[] = {rightFrontMotorChannel_ON_HIGH,0};
	rightFrontMotor.highRegOn = highRegOn;
	//write_i2c(AdaFruitAddr,2,highRegOn);
	//Value for lower 8 bit OFF register
	int lowRegOff[] = {rightFrontMotorChannel_OFF_LOW,on_count_low};//Sets the count values for the PWM to turn OFF
	rightFrontMotor.lowRegOn = lowRegOn;
	//write_i2c(AdaFruitAddr,2,lowRegOff);
	//Value for higher 8 bit OFF register
	int highRegOff[] = {rightFrontMotorChannel_OFF_HIGH,on_count_high};
	rightFrontMotor.highRegOff = highRegOff;
	//write_i2c(AdaFruitAddr,2,highRegOff);

	return rightFrontMotor;
}

//Sets the PWM for the bucket drum motor (asssigned to PWM channel 5)
//Input is a percentage value
pwmRegisterData setBucketDrum(float PWM)
{
	int on_count = PWM * .01 * 4095;//Converts input to decimal and gets correct counter value
	int on_count_high = (on_count/256);//Isolates higher 4 bits of count value
	int on_count_low = (on_count & 255);//Isolates lower 8 bits of count value
	
	pwmRegisterData bucketDrum;
	
	//Value for lower 8 bit ON register
	int lowRegOn[] = {bucketDrumMotorChannel_ON_LOW,0};//ON values are set to 0 so the counter turns 		ON at count value of 0, or at the beginning of the PWM cycle
	bucketDrum.lowRegOn = lowRegOn;
	//write_i2c(AdaFruitAddr,2,lowRegOn);
	//Value for higher 8 bit register
	int highRegOn[] = {bucketDrumMotorChannel_ON_HIGH,0};
	bucketDrum.highRegOn = highRegOn;
	//write_i2c(AdaFruitAddr,2,highRegOn);
	//Value for higher 8 bit OFF register
	int lowRegOff[] = {bucketDrumMotorChannel_OFF_LOW,on_count_low};//Sets the count values for the PWM to turn OFF
	bucketDrum.lowRegOff = lowRegOff;
	//write_i2c(AdaFruitAddr,2,lowRegOn);

	int highRegOff[] = {bucketDrumMotorChannel_OFF_HIGH,on_count_high};
	bucketDrum.highRegOff = highRegOff;
	//write_i2c(AdaFruitAddr,2,highRegOff);
	
	return bucketDrum;
}

//Sets the PWM for the bucket drum motor (asssigned to PWM channel 2)
//Input is a percentage value
pwmRegisterData setLinearActuator(float PWM)
{
	int on_count = PWM * .01 * 4095;//Converts input to decimal and gets correct counter value
	int on_count_high = (on_count/256);//Isolates higher 4 bits of count value
	int on_count_low = (on_count & 255);//Isolates lower 8 bits of count value
	
	pwmRegisterData linearActuator;
	
	//Value for lower 8 bit ON register
	int lowRegOn[] = {linearActuatorMotorChannel_ON_LOW,0};//ON values are set to 0 so the counter turns 		ON at count value of 0, or at the beginning of the PWM cycle
	linearActuator.lowRegOn = lowRegOn;
	//write_i2c(AdaFruitAddr,2,lowRegOn);
	
	//Value for higher 8 bit register
	int highRegOn[] = {linearActuatorMotorChannel_ON_HIGH,0};
	linearActuator.highRegOn = highRegOn;
	//write_i2c(AdaFruitAddr,2,highRegOn);
	//Value for higher 8 bit OFF register
	int lowRegOff[] = {linearActuatorMotorChannel_OFF_LOW,on_count_low};//Sets the count values for the PWM to turn OFF
	linearActuator.lowRegOff = lowRegOff;
	
	//write_i2c(AdaFruitAddr,2,lowRegOn);

	int highRegOff[] = {linearActuatorMotorChannel_OFF_HIGH,on_count_high};
	linearActuator.highRegOff = highRegOff;
	//write_i2c(AdaFruitAddr,2,highRegOff);
	
	return linearActuator;
}
