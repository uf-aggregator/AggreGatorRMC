#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#ifndef _LADAR_H_
#define _LADAR_H_ 

class Ladar {
	protected:
	//DATA MEMBERS
	std::vector<float> thetas, degrees, slopes;
	std::vector<std::pair<float, float> > coords;

	float angle_max, reference_angle;

	//ROS::NodeHandle nh, ROS::Subscriber sub, ROS::Publisher pub

	public:
	//CONSTRUCTORS
	Ladar(){}
	Ladar(int numOfSamples);
	~Ladar(){}

	//GETTER+SETTERS
	void setMaxAngle(float angle){ this->angle_max = angle;	}
	std::vector<float> getSlopes(std::vector<std::pair<float,float> > coordinates);
	std::vector<float> getAverageSlopes(std::vector<float> slopes);
	float getAverageSlope(std::vector<float> slopes);
	float getAverageSlope(std::vector<float> slopes, int startIndex, int endIndex);
	std::vector<float> getThetas() const { return thetas; }
	std::vector<std::pair<float, float> > getCoordinates() const { return coords; }

	//UTILITY METHODS
	float adjustTheta(float theta);
	std::string ghetto_to_string(float number);
	int drawCoordinates(std::vector<std::pair<float, float> > coordinates);

	//COORDINATE PROCESSING
	std::vector<std::pair<float, float> > getCoordinates(	float ranges[], int numOfSamples, 
															float angle_min, float angle_increment,
	                                            			float min_range, float max_range);
	std::vector<std::pair<float, float> > fivePointAverager(std::vector<std::pair<float, float> > original);
	std::string coordinatesToString(std::vector<std::pair<float, float> > coordinates);
	std::vector<int> findCorners(std::vector<float> slopes);
	void processSlopes();

	//PRINT METHODS
	void print(std::vector<float> choice, std::string type);
	void print(int choice);
	void printDegreesOnly();
	void printSlopesOnly();
	void printRadiansOnly();
	
	//CHECK METHODS
	bool forwardCheck();
	bool leftCheck();
	bool rightCheck();

};

#endif
