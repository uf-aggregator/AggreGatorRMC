#ifndef MOTO_UTILITY_H
#define MOTO_UTILITY_H

#include <std_msgs/builtin_int16.h>
#include <common_files/Motor.h>
#include <common_files/LinActMotor.h>
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
		static void publish_to_wheels(common_files::Motor msg);
		static void publish_to_actuators(common_files::LinActMotor msg);
	public:
		static void init();
		static void ros_init();
		static void stop_wheels();
		static void stop_actuators();
		static void stop();
		static void write(float leftMotorsVal, float rightMotorsVal);
		static void write(int mineVal, int mineDir, int dumpVal, int dumpDir);

		static void incWheelGear();
		static void decWheelGear();
		static double getWheelGear();
		static double getActuatorGear();
};

#endif
