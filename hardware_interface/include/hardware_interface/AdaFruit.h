/* by Daniel Kelly refactored by Andrew V
 * UF AggreGator
 * This header file contains declarations of the necessary functions to interface with Adafruit PWM generator
 */

#ifndef _ADAFRUIT_H_
#define _ADAFRUIT_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <iostream>
#include "GPIO.h"
//#include "i2c.h"
#include <unistd.h>
#include "ros/ros.h"
#include "common_msgs/AdaCmd.h"
#include "common_msgs/WriteI2C.h"
#include "common_msgs/WriteI2CRegister.h"
#include "hardware_interface/ReadI2CRegister.h"

//List of register addresses
enum regAddr
{
    //AdaFruit i2c Address
    AdaFruitAddr                        = 0x60,

    //Important setup register
    mode1                               = 0x00,
    mode2                               = 0x01,
    preScale                            = 0xFE,

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

//Structure which will be loaded with values that will be eventually written to the I2C bus
//These values will set the PWM channels to certain values to help control the motors.
struct pwmRegisterData
{
	int *lowRegOn;
	int *highRegOn;
	int *lowRegOff;
	int *highRegOff;
	int *initData;
};

struct PWM_setting
{
    u_int8_t low;
    u_int8_t high;
};


class AdaFruit {
    public:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher write_pub;
        ros::Publisher write_reg_pub;
        ros::ServiceClient read_register_svr;   //Reads I2C registers
        
        //Constructor + Destructor
        AdaFruit();
        ~AdaFruit(){

        }

        //Initialize
        pwmRegisterData AdaFruitInit();
        //Set relevant PWM channel for left front motor
        pwmRegisterData setLeftFrontMotor(float);
        //Set relevant PWM channel for left rear motor
        pwmRegisterData setLeftRearMotor(float);
        //Set relevant PWM channel for right rear motor
        pwmRegisterData setRightRearMotor(float);
        //Set relevant PWM channel for right front motor
        pwmRegisterData setRightFrontMotor(float);
        //Set relevant PWM channel for linear actuators
        pwmRegisterData setLinearActuator(float PWM);
        //Set relevant PWM channel for bucket drum
        pwmRegisterData setBucketDrum(float PWM);


        static PWM_setting convertSetpointToPWM(float set_point);
        static void WritePWMRegisters(PWM_setting pwm, regAddr base_addr);
        static void adaInitialize();
        static void setWheelMotorI2C(const common_msgs::AdaCmd& msg);
        static void setBucketMotorI2C(const common_msgs::AdaCmd& msg);
        static void setLinearActuatorI2C(const common_msgs::AdaCmd& msg);
        static void adaFruitCallBack(const common_msgs::AdaCmd& msg);

        int run();
};
#endif


