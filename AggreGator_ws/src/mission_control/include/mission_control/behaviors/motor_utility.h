#ifndef MOTO_UTILITY_H
#define MOTO_UTILITY_H

#include <std_msgs/builtin_int16.h>
#include <motor_controller/Motor.h>
#include <motor_controller/LinActMotor.h>
#include <cstdlib>
#include <ros/ros.h>
#include <string.h>
#include <queue>

class motor_utility {
	private:
		static unsigned int delay_queue_size;
		static double wheel_motor_gear;
		static double actuator_motor_gear;
		static double delay_time;
		static double send_time;

		static ros::Time prev_write_time;

		motor_utility();
		static void publish_to_wheels(motor_controller::Motor msg);
		static void publish_to_actuators(motor_controller::LinActMotor msg);
		
	public:
		static void init();
		static void ros_init();
		static void stop_wheels();
		static void stop_actuators();
		static void stop();
		static void write(float leftMotorsVal, float rightMotorsVal);
		static void write(int actuatorVal, int actuatorDir);

		static void incWheelGear();
		static void decWheelGear();
		static double getWheelGear();
		static double getActuatorGear();
		static double getDelay();
		static void setDelayTime(double newDelay);
};

#endif