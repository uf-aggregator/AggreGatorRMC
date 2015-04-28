#include "motor_utility.h"

/*CONSTRUCTORS================================*/
motor_utility::motor_utility(){ }

/*METHODS=====================================*/
void motor_utility::publish_to_wheels(common_files::Motor msg){
	if(!ros::isInitialized()) ros_init();
	ros::NodeHandle nh;
	ROS_INFO("Publish to wheels:%f, %f", msg.left, msg.right);

	ros::Publisher pub = nh.advertise<common_files::Motor>("motor_rc", 1000);
	pub.publish(msg);
	ros::spinOnce();
}

void motor_utility::publish_to_actuators(common_files::LinActMotor msg){
	if(!ros::isInitialized()) ros_init();
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<common_files::LinActMotor>("linear_actuator_rc",1000);
	pub.publish(msg);
}

void motor_utility::ros_init(){
	int argc; char** argv;
	ros::init(argc, argv, "motor_utility_class");
}

void motor_utility::stop_wheels(){
	common_files::Motor motor_msg;
	motor_msg.left = 0;
	motor_msg.right = 0;

	publish_to_wheels(motor_msg);
}

void motor_utility::stop_actuators(){
	common_files::LinActMotor actuator_msg;
	actuator_msg.mining_motorVal = 0;
	actuator_msg.dumping_motorVal = 0;
	publish_to_actuators(actuator_msg);
}

void motor_utility::stop(){
	stop_wheels();
	stop_actuators();
}

void motor_utility::write(float leftMotorsVal, float rightMotorsVal){
	float leftVal = leftMotorsVal * wheel_motor_gear;
	float rightVal = rightMotorsVal * wheel_motor_gear;

	common_files::Motor motor_msg;
	motor_msg.left = leftVal;
	motor_msg.right = rightVal;

	publish_to_wheels(motor_msg);
}

void motor_utility::write(int mineVal, int mineDir, int dumpVal, int dumpDir){
	common_files::LinActMotor actuator_msg;
	int value_to_write = (mineVal - 1) * -16383 * mineDir * actuator_motor_gear;
	actuator_msg.mining_motorVal = value_to_write;
	actuator_msg.dumping_motorVal = 0;
	publish_to_actuators(actuator_msg);
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
