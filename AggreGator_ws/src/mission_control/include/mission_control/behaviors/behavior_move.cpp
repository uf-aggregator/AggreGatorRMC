#include "behavior_move.h"

/*CONSTRUCTORS================================*/
behavior_move::behavior_move(){ }


/*METHODS=====================================*/
void behavior_move::publish(motor_controller::Motor msg){
	ros_init();
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<motor_controller::Motor>("motor_rc", 1000);
	pub.publish(msg);
}

void behavior_move::ros_init(){
	int argc; char** argv;
	ros::init(argc, argv, "behavior_dump_class");
}

void behavior_move::stop(){
	motor_controller::Motor motor_msg;
	motor_msg.leftFront_motorVal = 0;
	motor_msg.rightFront_motorVal = 0;
	motor_msg.rightRear_motorVal = 0;
	motor_msg.leftRear_motorVal = 0;

	publish(motor_msg);
}

void behavior_move::write(float leftMotorsVal, float rightMotorsVal){
	ros::Time current_time = ros::Time::now();
	ros::Duration send_freq(send_time);
	
	if(prev_write_time - current_time > send_freq){
		prev_write_time = current_time;

		float leftVal = leftMotorsVal * motor_gear;
		float rightVal = rightMotorsVal * motor_gear;

		motor_controller::Motor motor_msg;
		motor_msg.leftFront_motorVal = leftVal;
		motor_msg.leftRear_motorVal = leftVal;
		motor_msg.rightFront_motorVal = rightVal;
		motor_msg.rightRear_motorVal = rightVal;

		publish(motor_msg);
	}
}

void behavior_move::incGear(){
	if(motor_gear < 1.0f){
		motor_gear += 0.1f;
	}
}

void behavior_move::decGear(){
	if(motor_gear > 0.1f){
		motor_gear -= 0.1f;
	}
}

double behavior_move::getGear(){
	return motor_gear;
}

double behavior_move::getDelay(){
	return delay_time;
}

void behavior_move::setDelayTime(double newDelay){
	delay_time = newDelay;
}

/*DATA MEMBERS================================*/
//initialize primitive types
unsigned int behavior_move::delay_queue_size = 0;
double behavior_move::motor_gear = 0.7f;
double behavior_move::delay_time = 2;
double behavior_move::send_time = 0.1; //ten messages / minute
ros::Time behavior_move::prev_write_time = ros::Time::now();