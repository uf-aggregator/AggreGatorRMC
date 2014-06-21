#include <iostream>
#include <string>
#include "ladar/SDL/SDL.h"
#include "ros/ros.h"
#include "ladar/ladar.h"
#include "ladar/localization.h"
#include "ladar/draw.h"
#include "ladar/conversions.h"
#include "ladar/processed_data.h"

void scanCallback(const sensor_msgs::LaserScan laser){
    //Prep arguments from LaserScan message
	Ladar *ladar = new Ladar();

    int numSample = laser.ranges.size();
    float ranges[numSample];
    for(int i = 0; i < numSample; i++){
        ranges[i] = laser.ranges[i];
    }
    float angle_min = laser.angle_min;
    float angle_increment = laser.angle_increment;
    float min_range = laser.range_min;
    float max_range = laser.range_max;

    //instantiate classes
    Conversions *convert = new Conversions();

    //process Ladar data
    std::vector<std::pair<float, float> > coordinates = ladar->getCoordinates(ranges, numSample, angle_min, angle_increment, min_range, max_range);
    
    //do something with the data

    usleep(1000000);
}

/*================================================
 *Main Method
 *================================================*/
int main(int argc, char **argv){
	ros::init(argc, argv, "ladar_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);
	ros::spin();
	return 0;
}
