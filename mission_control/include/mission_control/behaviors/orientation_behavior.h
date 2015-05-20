#ifndef ORIENTATION_BEHAVIOR_H 
#define ORIENTATION_BEHAVIOR_H

#include <std_msgs/Float32.h>
#include "common_files/Gyro.h"
#include "common_files/Centroid.h"

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
		static float centroid_x, centroid_y;

		/* methods */
		static void updateAngle();
		static void turn(int degrees); //uses mpu

		/* callbacks */
		static void orientAngleCallback(const std_msgs::Float32::ConstPtr& msg);
		static void centroidCallback(const common_files::Centroid::ConstPtr& msg);

		/* orient by LEDs */
		static void turn();
		static void find();
		static void orientToLEDs();

		/* orient by centroid */
		static void orientByCentroid();
		static void turnIfFacingLidar();
};

#endif