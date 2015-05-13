#ifndef MOTO_UTILITY_H
#define MOTO_UTILITY_H

#include <std_msgs/builtin_int16.h>
#include <common_files/Drive.h>
#include <common_files/Ladder.h>
#include <common_files/Bucket.h>
#include <cstdlib>
#include <ros/ros.h>
#include <string.h>
#include <queue>

class motor_utility {
	private:
		static unsigned int delay_queue_size;
		static double wheel_motor_gear;
		static double actuator_motor_gear;

		motor_utility();
		static void publish_to_wheels(common_files::Drive msg);
		static void publish_to_ladder(common_files::Ladder msg);
		static void publish_to_bucket(common_files::Bucket msg);
	public:
		static void init();
		static void ros_init();
		static void stop_wheels();
		static void stop_bucket();
		static void stop_ladder();
		static void stop();
		static void write(float leftMotorsVal, float rightMotorsVal);
		static void write(float mineLift, float mineConv, float dumpLift, float dumpVal);
		static void write_to_ladder(float mineLift, float mineConv);
		static void write_to_bucket(float dumpLift, float dumpVal);

		static void incWheelGear();
		static void decWheelGear();
		static double getWheelGear();
		static double getActuatorGear();
};

#endif
