#ifndef ORIENTATION_BEHAVIOR_H 
#define ORIENTATION_BEHAVIOR_H

#include <std_msgs/Float32.h>
#include "common_files/Gyro.h"

enum Direction {
	NONE = -1,
	LEFT = 0,
	RIGHT
};

class OrientationBehavior {
	private:
		OrientationBehavior(){}
		~OrientationBehavior(){}

	public:
		static Direction lastSeen; //-1 null, 0 left, 1 right
		static float x1, y1; //first pair of coords
		static float x2, y2; //second pair of coords
		static float width, height; //resolution of the camera
		static float angle;

		/* methods */
		static void updateAngle();

		/* callbacks */
		static void orientAngleCallback(const std_msgs::Float32::ConstPtr& msg);

		/* orient by LEDs */
		static void turn();
		static void find();
		static void orientToLEDs();

		/* orient by mpu */
		static void turn(int degrees);
};

#endif