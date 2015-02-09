/*Daniel Kelly
UF AggreGator
	This contains definitions for the necessary interfacing functions to the Adafruit PWM generator, which will help control the motors
	*/
#include "AdaFruit.h"

using namespace std;

AdaFruit::AdaFruit(){

}

//Initializes the AdaFruit PWM generator by setting the MODE1 register accordingly.
pwmRegisterData AdaFruit::AdaFruitInit()
{
	pwmRegisterData initialize;
	//init_i2c(); //Initializes the I2C bus

	int data[] = {0x0,0b00000001}; //Leave everything in MODE1 as default, except turn off SLEEP bit
	initialize.initData = data;
	return initialize;
	
	//write_i2c(AdaFruitAddr,2,data); //Writes above data to the Adafruit

}


//Sets the PWM for the left front motor (assigned to PWM channel 3)
//Input is a percentage value
pwmRegisterData AdaFruit::setLeftFrontMotor(float PWM)
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
pwmRegisterData AdaFruit::setLeftRearMotor(float PWM)
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
pwmRegisterData AdaFruit::setRightRearMotor(float PWM)
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
pwmRegisterData AdaFruit::setRightFrontMotor(float PWM)
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
pwmRegisterData AdaFruit::setBucketDrum(float PWM)
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
pwmRegisterData AdaFruit::setLinearActuator(float PWM)
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


/*
 *  Converts a value between 0.0 and 100.0 to an appropriate
 *      setting on the ada fruit. It converts then
 *  Conversion:
 *       / 100         Scale to 0 to 1
 *      * 4095         Scale to a 12bit number
 */
PWM_setting AdaFruit::convertSetpointToPWM(float set_point)
{
    PWM_setting setting;

    //Scale set_point
    set_point *= 40.95;

    setting.high = (int)(set_point / 256) & 0x0F;   //Right shift 8 bits (divide by 256) then convert to int8
    setting.low = (int)(set_point) & 0xFF;          //Convert to int take lower 8 bits

    return setting;
}

/*
 *  Writes the pwm generater registers
 *  Writes 0 to the on registers
 *  Writes PWM_setting values to off registers
 *
 *  Inputs:
 *      pwm is the pwm settings
 *      base_addr is the addr of the low on register
 */
void AdaFruit::WritePWMRegisters(PWM_setting pwm, regAddr base_addr)
{
	AdaFruit *af;
	
    //Create and setup message object
    common_msgs::WriteI2CRegister msg;
    msg.addr = AdaFruitAddr;
    msg.reg = base_addr;

    //Debug
    ROS_INFO("Convfigure PWM: Low_off: %i, High_off: %i",
	pwm.low, pwm.high); 

    //Write 0 to on registers
    msg.data.push_back(0);
    af->write_reg_pub.publish(msg);

    ++msg.reg;
    af->write_reg_pub.publish(msg);

    //Write to off registers
    ++msg.reg;
    msg.data[0] = pwm.low;
    af->write_reg_pub.publish(msg);

    ++msg.reg;
    msg.data[0] = pwm.high;
    af->write_reg_pub.publish(msg);
}

//Writes messages that initialize the AdaFruit hardware,
//by clearing all PWM channels and setting the necessary modes.
void AdaFruit::adaInitialize()
{
	AdaFruit *af;

	//ROS_INFO("PUBLISH");
    //Create init message
    common_msgs::WriteI2CRegister init;

    //Sets the addr field as the AdaFruit's address on the I2C bus & set register to mode1
    init.addr = AdaFruitAddr;
    init.reg = mode1;

    //Leave defaults turn off sleep
    init.data.push_back(0b1);

    //Publish the msg
    while(af->write_reg_pub.getNumSubscribers() == 0);//Prevents message from sending when publisher is not completely connected to subscriber.

    af->write_reg_pub.publish(init);

    //Sets prescale register to output PWM signal at 1.5 kHz
    init.reg = preScale;
    init.data.push_back(0x3);

    af->write_reg_pub.publish(init);
	//Define and fill in a service request
    hardware_interface::ReadI2CRegister read;
    read.request.addr = AdaFruitAddr;
    read.request.size = 1;

	for(int i = 0; i <= 4; ++i)
	{
		
    		read.request.reg = mode1;
 		//Send request
	    if(af->read_register_svr.call(read))
	    {
	       //	ROS_INFO("Reg: %i, Value: %i", i, read.response.data[0]);
	    }
	    else
	    {
		//Unsussesful read!!!
		ROS_ERROR("Reg: %i, unreadable!", i);
	    }
	}
}

//Function writes and sends out messages of WriteI2C type. Contains data that sets the PWM value on the Adafruit hardware
void AdaFruit::setWheelMotorI2C(const common_msgs::AdaCmd& msg)
{  
    //Some reusable variables
    PWM_setting pwm;                                    //For holding PWM settings

    //Left Front motor
    pwm = AdaFruit::convertSetpointToPWM(msg.value[0]);
    AdaFruit::WritePWMRegisters(pwm, leftFrontMotorChannel_ON_LOW);

    //Left rear motor
    pwm = AdaFruit::convertSetpointToPWM(msg.value[1]);
    AdaFruit::WritePWMRegisters(pwm, leftRearMotorChannel_ON_LOW);

    //Right rear motor
    pwm = AdaFruit::convertSetpointToPWM(msg.value[2]);
    AdaFruit::WritePWMRegisters(pwm, rightRearMotorChannel_ON_LOW);

    //Right Front motor
    pwm = AdaFruit::convertSetpointToPWM(msg.value[3]);
    AdaFruit::WritePWMRegisters(pwm, rightFrontMotorChannel_ON_LOW);
}

//Function that writes data to WriteI2C message, which will be used to set the PWM onthe AdaFruit hardware for the bucket drum motor
void AdaFruit::setBucketMotorI2C(const common_msgs::AdaCmd& msg)
{
    PWM_setting pwm = AdaFruit::convertSetpointToPWM(msg.value[0]);
    AdaFruit::WritePWMRegisters(pwm, bucketDrumMotorChannel_ON_LOW);
}

//Function that writes data to WriteI2C message, which will be used to set the PWM onthe AdaFruit hardware for the linear actuators
void AdaFruit::setLinearActuatorI2C(const common_msgs::AdaCmd& msg)
{	
    PWM_setting pwm = AdaFruit::convertSetpointToPWM(msg.value[0]);
    AdaFruit::WritePWMRegisters(pwm, linearActuatorMotorChannel_ON_LOW);
}

//Callback function
void AdaFruit::adaFruitCallBack(const common_msgs::AdaCmd& msg)
{

		//ROS_INFO("Receiving message: [%i] ",msg.device);

        //Checks device marker in incoming message
        switch(msg.device)
		{
		case common_msgs::AdaCmd::wheelMotors: //If wheelMotors, publish data relating to the wheel motors
			AdaFruit::setWheelMotorI2C(msg);
			break;
		case common_msgs::AdaCmd::bucketDrum: //If bucketDrum,publish data relating to the bucket drum motor
			AdaFruit::setBucketMotorI2C(msg);
			break;
		case common_msgs::AdaCmd::linearActuator: //If linearActuator, publish data relating to the linear actuators
			AdaFruit::setLinearActuatorI2C(msg);
			break;
		default:
            ROS_ERROR("Received message does not contain a legitimate device parameter"); //If no known device is in incoming message, output an error message
			break;
		}
}

int AdaFruit::run(){
    //Setup publishers and subscribers
	sub = n.subscribe("adaFruit", 1000, AdaFruit::adaFruitCallBack);
	write_pub = n.advertise<common_msgs::WriteI2C>("write_i2c",1000);
    write_reg_pub = n.advertise<common_msgs::WriteI2CRegister>("write_i2c_register", 1000);

    read_register_svr = n.serviceClient<hardware_interface::ReadI2CRegister>("read_i2c_register");
	
    ros::spinOnce();

    //Initialize the adafruit node
    adaInitialize();

    ros::spin();
	//return success status
	return 0;
}
