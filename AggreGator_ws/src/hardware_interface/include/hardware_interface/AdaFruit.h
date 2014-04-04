#ifndef _ADAFRUIT_H_
#define _ADAFRUIT_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <iostream>
#include "GPIO.h"
#include "i2c.h"
#include <unistd.h>
//AdaFruit interfacing functions

//List of register addresses
enum regAddr
{
 AdaFruitAddr 			= 0x60,
 leftFrontMotorChannel_ON_LOW   = 0x06,
 leftFrontMotorChannel_ON_HIGH,
 leftFrontMotorChannel_OFF_LOW,
 leftFrontMotorChannel_OFF_HIGH,
 leftRearMotorChannel_ON_LOW,
 leftRearMotorChannel_ON_HIGH,
 leftRearMotorChannel_OFF_LOW,
 leftRearMotorChannel_OFF_HIGH,
 rightRearMotorChannel_ON_LOW,
 rightRearMotorChannel_ON_HIGH, 
 rightRearMotorChannel_OFF_LOW,
 rightRearMotorChannel_OFF_HIGH,
 rightFrontMotorChannel_ON_LOW,
 rightFrontMotorChannel_ON_HIGH,
 rightFrontMotorChannel_OFF_LOW, 
 rightFrontMotorChannel_OFF_HIGH 
};

//Initialize
void AdaFruitInit();
//Set relevant PWM channel for left front motor
void setLeftFrontMotor(int);
//Set relevant PWM channel for left rear motor
void setLeftRearMotor(int);
//Set relevant PWM channel for right rear motor
void setRightRearMotor(int);
//Set relevant PWM channel for right front motor
void setRightFrontMotor(int);

#endif //_ADAFRUIT_H_
