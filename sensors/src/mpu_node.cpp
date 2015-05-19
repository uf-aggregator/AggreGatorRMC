/*
	this node polls the imu
*/

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "common_files/ReadI2C.h"
#include "common_files/WriteI2C.h"
#include "common_files/Gyro.h"

#define MPU_ADDR 0x68
#define MAX_INT 32767
#define MAX_GYRO 250 //assuming default gyroscope config

ros::Publisher write_i2c;
ros::ServiceClient read_i2c;
ros::Publisher gyro;

int16_t x_acc = 0;
int16_t y_acc = 0;
int16_t z_acc = 0;
uint16_t temperature = 0;
int16_t x_gyro = 0;
int16_t y_gyro = 0;
int16_t z_gyro = 0;

common_files::Gyro last_gyro;


void mpuCallback(const ros::TimerEvent&){
	//to registers from mpu, you must first specify a starting point
	//the starting register is x acceleration high bits, at location 0x3B

	//we want to read 14 registers
		//each value is 2 registers (16 bits)
		//in this order:
			//0-1: X acceleration
			//2-3: Y acceleration
			//4-5: Z acceleration
			//6-7: Temperature
			//8-9: X gyro
			//10-11: Y gyro
			//12-13: Z gyro

	//tell the mpu you want to start at register 0x3B
	common_files::WriteI2C mpu_reg_msg;
	mpu_reg_msg.addr = MPU_ADDR;
	mpu_reg_msg.data.push_back(0x3B); //x acceleration high reg
	write_i2c.publish(mpu_reg_msg);

	//now read 14 registers starting at 0x3B
    common_files::ReadI2C mpu_srv;
	mpu_srv.request.addr = MPU_ADDR;
	mpu_srv.request.size = 14;

    if(read_i2c.call(mpu_srv)){
		x_acc = (mpu_srv.response.data[0] << 8) | mpu_srv.response.data[1];
		y_acc = (mpu_srv.response.data[2] << 8) | mpu_srv.response.data[3];
		z_acc = (mpu_srv.response.data[4] << 8) | mpu_srv.response.data[5];
		temperature = (mpu_srv.response.data[6] << 8) | mpu_srv.response.data[7];
		x_gyro = (mpu_srv.response.data[8] << 8) | mpu_srv.response.data[9];
		y_gyro = (mpu_srv.response.data[10] << 8) | mpu_srv.response.data[11];
		z_gyro = (mpu_srv.response.data[12] << 8) | mpu_srv.response.data[13];

		//convert the gyro adc data to degrees per second (dps)
		last_gyro.x = (((float)x_gyro)/MAX_INT)*(250.0);
		last_gyro.y = (((float)y_gyro)/MAX_INT)*(250.0);
		last_gyro.z = (((float)z_gyro)/MAX_INT)*(250.0);
		gyro.publish(last_gyro);	

	//	ROS_INFO("ACCELERATION: %d, %d, %d", x_acc, y_acc, z_acc);
	//	ROS_INFO("Temperature: %d", temperature);
	//	ROS_INFO("GYRO: %d, %d, %d", x_gyro, y_gyro, z_gyro);
	} else {
        ROS_INFO("Failed to call read_i2c teensy service");
    }
}

void rosoutCallback(const ros::TimerEvent&){
	ROS_INFO("ACCELERATION: %d, %d, %d", x_acc, y_acc, z_acc);
    ROS_INFO("Temperature: %d", temperature);
    ROS_INFO("GYRO: %d, %d, %d", x_gyro, y_gyro, z_gyro);
	ROS_INFO("%f, %f, %f degrees per second", last_gyro.x, last_gyro.y, last_gyro.z);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "mpu_node"); 
	ros::NodeHandle n;
		
	write_i2c = n.advertise<common_files::WriteI2C>("write_i2c", 1);
	read_i2c = n.serviceClient<common_files::ReadI2C>("read_i2c");
	gyro = n.advertise<common_files::Gyro>("gyro", 1);	

	ros::spinOnce();
	
	ros::Duration(1).sleep();

	//initialize MPU 
	common_files::WriteI2C mpu_wake_msg;
	mpu_wake_msg.addr = MPU_ADDR;
	mpu_wake_msg.data.push_back(0x6B); //PWR_MGMT_1 reg
	mpu_wake_msg.data.push_back(0); //set to zero (wakes MPU)
	write_i2c.publish(mpu_wake_msg);
	
	ros::spinOnce();

	ROS_INFO("MPU6050 awake");
	ros::Duration(1).sleep();

	//read mpu every tenth of a second
    ros::Timer mpu_timer = n.createTimer(ros::Duration(0.01), mpuCallback);

    ros::Timer ros_out_timer = n.createTimer(ros::Duration(.25), rosoutCallback);

	while(ros::ok()){
		ros::spinOnce();
	}
}
