#include "motor_utility.h"

/*CONSTRUCTORS================================*/
motor_utility::motor_utility(){ }

/*METHODS=====================================*/
void motor_utility::publish_to_wheels(motor_controller::Motor msg){
	ros_init();
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<motor_controller::Motor>("motor_rc", 1000);
	pub.publish(msg);
}

void motor_utility::publish_to_actuators(motor_controller::LinActMotor msg){
	ros_init();
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<motor_controller::LinActMotor>("linear_actuator_rc",1000);
	pub.publish(msg);
}

void motor_utility::ros_init(){
	int argc; char** argv;
	ros::init(argc, argv, "motor_utility_class");
}

void motor_utility::stop_wheels(){
	motor_controller::Motor motor_msg;
	motor_msg.leftFront_motorVal = 0;
	motor_msg.rightFront_motorVal = 0;
	motor_msg.rightRear_motorVal = 0;
	motor_msg.leftRear_motorVal = 0;

	publish_to_wheels(motor_msg);
}

void motor_utility::stop_actuators(){
	motor_controller::LinActMotor actuator_msg;
	actuator_msg.mining_motorVal = 0;
	actuator_msg.dumping_motorVal = 0;
	publish_to_actuators(actuator_msg);
}

void motor_utility::stop(){
	stop_wheels();
	stop_actuators();
}

void motor_utility::write(float leftMotorsVal, float rightMotorsVal){
	ros::Time current_time = ros::Time::now();
	ros::Duration send_freq(send_time);
	
	if(prev_write_time - current_time > send_freq){
		prev_write_time = current_time;

		float leftVal = leftMotorsVal * wheel_motor_gear;
		float rightVal = rightMotorsVal * wheel_motor_gear;

		motor_controller::Motor motor_msg;
		motor_msg.leftFront_motorVal = leftVal;
		motor_msg.leftRear_motorVal = leftVal;
		motor_msg.rightFront_motorVal = rightVal;
		motor_msg.rightRear_motorVal = rightVal;

		publish_to_wheels(motor_msg);
	}
}

void motor_utility::write(int mineVal, int mineDir, int dumpVal, int dumpDir){
	ros::Time current_time = ros::Time::now();
	ros::Duration send_freq(send_time);
	
	if(prev_write_time - current_time > send_freq){
		prev_write_time = current_time;

		motor_controller::LinActMotor actuator_msg;
		int value_to_write = (mineVal - 1) * -16383 * mineDir * actuator_motor_gear;
		actuator_msg.mining_motorVal = value_to_write;
		actuator_msg.dumping_motorVal = 0;
		publish_to_actuators(actuator_msg);
	}
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

double motor_utility::getDelay(){
	return delay_time;
}

void motor_utility::setDelayTime(double newDelay){
	delay_time = newDelay;
}

/*DATA MEMBERS================================*/
//initialize primitive types
unsigned int motor_utility::delay_queue_size = 0;
double motor_utility::wheel_motor_gear = 0.7f;
double motor_utility::actuator_motor_gear = 0.7f;
double motor_utility::delay_time = 2;
double motor_utility::send_time = 0.1; //ten messages / minute
ros::Time motor_utility::prev_write_time = ros::Time::now();