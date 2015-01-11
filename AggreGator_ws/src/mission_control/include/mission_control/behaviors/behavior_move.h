#ifndef BEHAVIOR_DUMP_H
#define BEHAVIOR_DUMP_H

#include <std_msgs/builtin_int16.h>
#include <motor_controller/Motor.h>
#include <cstdlib>
#include <ros/ros.h>
#include <string.h>
#include <queue>

class behavior_move {
	private:
		static unsigned int delay_queue_size;
		static double motor_gear;
		static double delay_time;
		static double send_time;

		static ros::Time prev_write_time;

		behavior_move();
		static void publish(motor_controller::Motor msg);
		
	public:
		static void init();
		static void ros_init();
		static void stop();
		static void write(float leftMotorsVal, float rightMotorsVal);

		static void incGear();
		static void decGear();
		static double getGear();
		static double getDelay();
		static void setDelayTime(double newDelay);
};

#endif