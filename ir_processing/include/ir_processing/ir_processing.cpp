#include <ros/ros.h>

#include "ir_processing.h"
#include "common_files/ReadI2C.h"

/*CONSTRUCTOR===============================*/
IrProcessing::IrProcessing(){}

/*INIT STATIC VALUES========================*/
const char* IrProcessing::serviceNm = "read_i2c";
const float IrProcessing::i2c_max = 1024.0;
const int IrProcessing::i2c_addr = 21;  //
const int IrProcessing::i2c_size = 128; //bytes, total: 1024 bits 
const float IrProcessing::ir_max_out = 5.3; //max output voltage
const float IrProcessing::ir_min_dist = 20.0; //cm
const float IrProcessing::ir_max_dist = 150.0; //cm


/*METHODS===============================*/

uint IrProcessing::getValue(){
	//start the necessities
	int argc; char** argv;
	ros::init(argc, argv, "init_ir_processing");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<common_files::ReadI2C>(serviceNm);
	ros::Publisher pub = n.advertise<common_files::I2CRead>("write_i2c",1000);

	//WRITING the command byte
	common_files::I2CRead msg;
	msg.addr = i2c_addr; //must be 15 < some value < 255, chose a legal value
	pub.publish(msg);

	//safety sleep to provide some assurance the value is written
	ros::Duration(0.1).sleep(); //duration is in seconds

	//READING
	//prep the service parameters
	common_files::ReadI2C data;
	data.request.addr = addr;
	data.request.size = size;

	//fetch the value from the service
	if(client.call(data)){
		return data.response.data[0];
	}
	else {
		ROS_ERROR("Failed to call the service %s", serviceNm);
		return 0;
	}
}

int IrProcessing::getCentimeters(){
	int value = (int)getValue();
	int cm = getCentimeterI2CMap(value);
	return cm;
}

//datasheet for IR: https://www.pololu.com/file/0J156/gp2y0a02yk_e.pdf
int IrProcessing::getCentimeterI2CMap(int value){
	int distance_in_cm;
	float voltage = (value/i2c_max) * ir_max_out;

	//using the following eqn:
	//	f(y) = 

	return distance_in_cm;
}

float IrProcessing::getMeters(){
	return 1000 * (float)getCentimeters();
}

float IrProcessing::getFeet(){
	return 0.3048 * getMeters();
}