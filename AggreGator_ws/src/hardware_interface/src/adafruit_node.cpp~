#include "ros/ros.h"
//#include "AdaFruit.h"
#include "motor_controller/AdaCmd.h"
#include "hardware_interface/WriteI2C.h"
#include "hardware_interface/WriteI2CRegister.h"
#include "hardware_interface/ReadI2CRegister.h"

ros::Subscriber sub;
ros::Publisher write_pub;
ros::Publisher write_reg_pub;

ros::ServiceClient read_register_svr;   //Reads I2C registers


//List of register addresses
enum regAddr
{
    //AdaFruit i2c Address
    AdaFruitAddr                        = 0x60,

    //Important setup register
    mode1                               = 0x00,
    mode2                               = 0x01,
    preScale				= 0xFE,

    //Motor Control Channels
    leftFrontMotorChannel_ON_LOW        = 0x12,
    leftFrontMotorChannel_ON_HIGH,
    leftFrontMotorChannel_OFF_LOW,
    leftFrontMotorChannel_OFF_HIGH,

    leftRearMotorChannel_ON_LOW         = 0x06,
    leftRearMotorChannel_ON_HIGH,
    leftRearMotorChannel_OFF_LOW,
    leftRearMotorChannel_OFF_HIGH,

    rightRearMotorChannel_ON_LOW        = 0x16,
    rightRearMotorChannel_ON_HIGH,
    rightRearMotorChannel_OFF_LOW,
    rightRearMotorChannel_OFF_HIGH,

    rightFrontMotorChannel_ON_LOW       = 0x0A,
    rightFrontMotorChannel_ON_HIGH,
    rightFrontMotorChannel_OFF_LOW,
    rightFrontMotorChannel_OFF_HIGH,

    bucketDrumMotorChannel_ON_LOW       = 0x1A,
    bucketDrumMotorChannel_ON_HIGH,
    bucketDrumMotorChannel_OFF_LOW,
    bucketDrumMotorChannel_OFF_HIGH,

    linearActuatorMotorChannel_ON_LOW   = 0x0E,
    linearActuatorMotorChannel_ON_HIGH,
    linearActuatorMotorChannel_OFF_LOW,
    linearActuatorMotorChannel_OFF_HIGH
};

struct PWM_setting
{
    u_int8_t low;
    u_int8_t high;
};

/*
 *  Converts a value between 0.0 and 100.0 to an appropriate
 *      setting on the ada fruit. It converts then
 *  Conversion:
 *       / 100         Scale to 0 to 1
 *      * 4095         Scale to a 12bit number
 */
PWM_setting convertSetpointToPWM(float set_point)
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
void WritePWMRegisters(PWM_setting pwm, regAddr base_addr)
{
    //Create and setup message object
    hardware_interface::WriteI2CRegister msg;
    msg.addr = AdaFruitAddr;
    msg.reg = base_addr;

    //Debug
    ROS_INFO("Convfigure PWM: Low_off: %i, High_off: %i",
	pwm.low, pwm.high); 

    //Write 0 to on registers
    msg.data.push_back(0);
    write_reg_pub.publish(msg);

    ++msg.reg;
    write_reg_pub.publish(msg);

    //Write to off registers
    ++msg.reg;
    msg.data[0] = pwm.low;
    write_reg_pub.publish(msg);

    ++msg.reg;
    msg.data[0] = pwm.high;
    write_reg_pub.publish(msg);
}

//Writes messages that initialize the AdaFruit hardware,
//by clearing all PWM channels and setting the necessary modes.
void adaInitialize()
{
	//ROS_INFO("PUBLISH");
    //Create init message
    hardware_interface::WriteI2CRegister init;

    //Sets the addr field as the AdaFruit's address on the I2C bus & set register to mode1
    init.addr = AdaFruitAddr;
    init.reg = mode1;

    //Leave defaults turn off sleep
    init.data.push_back(0b1);

    //Publish the msg
    while(write_reg_pub.getNumSubscribers() == 0);//Prevents message from sending when publisher is not completely connected to subscriber.

    write_reg_pub.publish(init);

    //Sets prescale register to output PWM signal at 1.5 kHz
    init.reg = preScale;
    init.data.push_back(0x3);

    write_reg_pub.publish(init);
	//Define and fill in a service request
    hardware_interface::ReadI2CRegister read;
    read.request.addr = AdaFruitAddr;
    read.request.size = 1;

	for(int i = 0; i <= 4; ++i)
	{
		
    		read.request.reg = mode1;
 		//Send request
	    if(read_register_svr.call(read))
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
void setWheelMotorI2C(const motor_controller::AdaCmd& msg)
{  
    //Some reusable variables
    PWM_setting pwm;                                    //For holding PWM settings

    //Left Front motor
    pwm = convertSetpointToPWM(msg.value[0]);
    WritePWMRegisters(pwm, leftFrontMotorChannel_ON_LOW);

    //Left rear motor
    pwm = convertSetpointToPWM(msg.value[1]);
    WritePWMRegisters(pwm, leftRearMotorChannel_ON_LOW);

    //Right rear motor
    pwm = convertSetpointToPWM(msg.value[2]);
    WritePWMRegisters(pwm, rightRearMotorChannel_ON_LOW);

    //Right Front motor
    pwm = convertSetpointToPWM(msg.value[3]);
    WritePWMRegisters(pwm, rightFrontMotorChannel_ON_LOW);
}

//Function that writes data to WriteI2C message, which will be used to set the PWM onthe AdaFruit hardware for the bucket drum motor
void setBucketMotorI2C(const motor_controller::AdaCmd& msg)
{
    PWM_setting pwm = convertSetpointToPWM(msg.value[0]);
    WritePWMRegisters(pwm, bucketDrumMotorChannel_ON_LOW);
}

//Function that writes data to WriteI2C message, which will be used to set the PWM onthe AdaFruit hardware for the linear actuators
void setLinearActuatorI2C(const motor_controller::AdaCmd& msg)
{	
    PWM_setting pwm = convertSetpointToPWM(msg.value[0]);
    WritePWMRegisters(pwm, linearActuatorMotorChannel_ON_LOW);
}

//Callback function
void adaFruitCallBack(const motor_controller::AdaCmd& msg)
{

		//ROS_INFO("Receiving message: [%i] ",msg.device);

        //Checks device marker in incoming message
        switch(msg.device)
		{
		case motor_controller::AdaCmd::wheelMotors: //If wheelMotors, publish data relating to the wheel motors
			setWheelMotorI2C(msg);
			break;
		case motor_controller::AdaCmd::bucketDrum: //If bucketDrum,publish data relating to the bucket drum motor
			setBucketMotorI2C(msg);
			break;
		case motor_controller::AdaCmd::linearActuator: //If linearActuator, publish data relating to the linear actuators
			setLinearActuatorI2C(msg);
			break;
		default:
            ROS_ERROR("Received message does not contain a legitimate device parameter"); //If no known device is in incoming message, output an error message
			break;
		}
}

int main(int argc, char** argv)
{    
    //Initilize ros
    ros::init(argc, argv, "adafruit_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Setup publishers and subscribers
	sub = n.subscribe("adaFruit", 1000, adaFruitCallBack);
	write_pub = n.advertise<hardware_interface::WriteI2C>("write_i2c",1000);
    write_reg_pub = n.advertise<hardware_interface::WriteI2CRegister>("write_i2c_register", 1000);

read_register_svr = n.serviceClient<hardware_interface::ReadI2CRegister>("read_i2c_register");
	ros::spinOnce();

    //Initialize the adafruit node
    adaInitialize();


    ros::spin();
	
}
