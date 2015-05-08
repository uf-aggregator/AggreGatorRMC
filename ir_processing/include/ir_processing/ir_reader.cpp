#include <ros/ros.h>

#include "ir_reader.h"
#include "common_files/WriteI2C.h"
#include "common_files/ReadI2C.h"
#include "common_files/IRDistances.h"

/*CONSTRUCTOR===============================*/
IrReader::IrReader(int addr, int size){
	i2c_addr = addr;
	i2c_size = size;
}

/*INIT STATIC VALUES========================*/
const char* IrReader::serviceNm = "read_i2c";
const float IrReader::i2c_max = 1024.0;
const float IrReader::ir_max_out = 5.3; //max output voltage
const float IrReader::ir_min_dist = 20.0; //cm
const float IrReader::ir_max_dist = 150.0; //cm


/*INSTANCE METHODS===============================*/
uint IrReader::getValue(){
	return getValueOf(i2c_addr, i2c_size);
}

int IrReader::getCentimeters(){
	return getCentimetersOf(i2c_addr, i2c_size);
}

float IrReader::getMeters(){
	return getMetersOf(i2c_addr, i2c_size);
}

float IrReader::getFeet(){
	return getFeetOf(i2c_addr, i2c_size);
}


/*STATIC METHODS===============================*/
uint IrReader::getValueOf(int addr, int size){
	//start the necessities
	int argc; char** argv;
	ros::init(argc, argv, "init_ir_processing");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<common_files::ReadI2C>(serviceNm);
	ros::Publisher pub = n.advertise<common_files::WriteI2C>("write_i2c",1000);

	//WRITING the command byte
	common_files::WriteI2C msg;
	msg.addr = addr; //must be 15 < some value < 255, chose a legal value
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

int IrReader::getCentimetersOf(int addr, int size){
	int value = (int)getValueOf(addr, size);
	int cm = getCentimeterI2CMap(value);
	return cm;
}

float IrReader::getMetersOf(int addr, int size){
	return 1000 * (float)getCentimetersOf(addr, size);
}

float IrReader::getFeetOf(int addr, int size){
	return 0.3048 * getMetersOf(addr, size);
}

int IrReader::getMetersOf(int centimeters) {
	return 1000 * (float)centimeters;
}

int IrReader::getFeetOf(int centimeters) {
	return 0.3048 * getMetersOf(centimeters);
}

//datasheet for IR: https://www.pololu.com/file/0J156/gp2y0a02yk_e.pdf
int IrReader::getCentimeterI2CMap(int value){
	int distance_in_cm;
	float voltage = (value/i2c_max) * ir_max_out;

	//using the following eqn:
	//	y = mx + b
	float m = -2.35/135.0; 		//derived from datasheet graph
	float intercept = 3.011;	//derived from datasheet graph
	distance_in_cm = (voltage - intercept) / m;
	return distance_in_cm;
}

void IrReader::ros_init(){
	int argc; char** argv;
	ros::init(argc, argv, "ir_reader_class");
}

void IrReader::publishDistances(){
	if(!ros::isInitialized()) ros_init();

	ros::NodeHandle nh;
	common_files::IRDistances irds;

	ros::Publisher pub = nh.advertise<common_files::IRDistances>("", 100);
	
	pub.publish(irds);

	ros::spinOnce();
}
