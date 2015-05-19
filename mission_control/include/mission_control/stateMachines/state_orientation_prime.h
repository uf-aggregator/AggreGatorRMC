#ifndef  OrientationStatePrime_H
#define  OrientationStatePrime_H
 
#include "common_files/ReadLidar.h"
#include "../behaviors/motor_utility.h"
#include "common_files/Centroid.h"
#include "std_msgs/Float32.h"
#include"std_msgs/Int8.h"
#include <cstdlib>
#include <ros/ros.h>
#include <string>
using namespace std;
 
class OrientationStatePrime {
	
public:

	static common_files::Centroid LidarCoordinates;
	static std_msgs::Float32 GyroAngle;

	int x, y, Gyroscope;
	int Starting_Line;
	string Position;
	static double Initial_x;
	static double Initial_y;
	
	OrientationStatePrime();
	double Distance_Traveled();
	void Move_A_Bit();
	int OrientToStart();
	int Determine_Location( double x, double y);
	static void LidarCoordinatesCallback(const common_files::Centroid::ConstPtr& msg);
	static void GyroCallback(const std_msgs::Float32::ConstPtr& msg);
	
};
#endif
	
