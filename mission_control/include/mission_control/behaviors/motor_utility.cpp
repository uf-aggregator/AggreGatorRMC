#include "motor_utility.h"

/*CONSTRUCTORS================================*/
motor_utility::motor_utility(){ }

/*METHODS=====================================*/
void motor_utility::publish_to_wheels(common_files::Drive msg){
	if(!ros::isInitialized()) ros_init();
	ros::NodeHandle nh;
	ROS_INFO("Publish to wheels:%f, %f", msg.left, msg.right);

	ros::Publisher pub = nh.advertise<common_files::Drive>("drive_vals", 10);
	pub.publish(msg);
	ros::spinOnce();
}

void motor_utility::publish_to_ladder(common_files::Ladder msg) {
	if(!ros::isInitialized()) ros_init();
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<common_files::Ladder>("ladder_vals",10);
	pub.publish(msg);
}

void motor_utility::publish_to_bucket(common_files::Bucket msg) {
	if(!ros::isInitialized()) ros_init();
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<common_files::Bucket>("bucket_vals",10);
	pub.publish(msg);
}

void motor_utility::ros_init(){
	int argc; char** argv;
	ros::init(argc, argv, "motor_utility_class");
}

void motor_utility::stop_wheels(){
	common_files::Drive motor_msg;
	motor_msg.left = 0;
	motor_msg.right = 0;
	publish_to_wheels(motor_msg);
}

void motor_utility::stop_bucket(){
	common_files::Bucket bucket_msg;
	bucket_msg.lift = 0.0;
	bucket_msg.dump = 0.0;
	publish_to_bucket(bucket_msg);
}

void motor_utility::stop_ladder(){
	common_files::Ladder ladder_msg;
	ladder_msg.lift = 0.0;
	ladder_msg.conv = 0.0;
	publish_to_ladder(ladder_msg);
}

void motor_utility::stop(){
	stop_wheels();
	stop_bucket();
	stop_ladder();
}

void motor_utility::write(float leftMotorsVal, float rightMotorsVal){
	float leftVal = leftMotorsVal * wheel_motor_gear;
	float rightVal = rightMotorsVal * wheel_motor_gear;

	common_files::Drive motor_msg;
	motor_msg.left = leftVal;
	motor_msg.right = rightVal;

	publish_to_wheels(motor_msg);
}

void motor_utility::write(float mineLift, float mineConv, float dumpLift, float dumpVal){
	if(mineLift != 0.0 && mineConv != 0.0) write_to_ladder(mineLift, mineConv);
	if(dumpLift != 0.0 && dumpVal != 0.0) write_to_bucket(dumpLift, dumpVal);
}

void motor_utility::write_to_ladder(float lift, float conv){
	common_files::Ladder ladder_msg;
	ladder_msg.lift = lift;
	ladder_msg.conv = conv;

	publish_to_ladder(ladder_msg);
}

void motor_utility::write_to_bucket(float lift, float dump){
	common_files::Bucket bucket_msg;
	bucket_msg.lift = lift;
	bucket_msg.dump = dump;

	publish_to_bucket(bucket_msg);
}

void motor_utility::incWheelGear(){
	if(wheel_motor_gear < 1.0f){
		wheel_motor_gear += 0.1f;
	}
}

void motor_utility::decWheelGear(){
	if(wheel_motor_gear > 0.1f){
		wheel_motor_gear -= 0.1f;
	}
}

double motor_utility::getWheelGear(){
	return wheel_motor_gear;
}

double motor_utility::getActuatorGear(){
	return actuator_motor_gear;
}

/*DATA MEMBERS================================*/
//initialize primitive types
unsigned int motor_utility::delay_queue_size = 0;
double motor_utility::wheel_motor_gear = 0.7f;
double motor_utility::actuator_motor_gear = 0.7f;
