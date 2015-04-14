//PCL Converter
//This node takes sensor_msgs/LaserScan from the urg_node and converts them into PointCloud2 messages
//The class comes with includes a pre-filter function for filtering before the PCL conversion

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <vector>
#include <cmath>

laser_geometry::LaserProjection projector_; //used for the conversion

ros::Publisher output; //output for point clouds
ros::Subscriber input; //input for laser scans
ros::Subscriber command; //input for commands

sensor_msgs::LaserScan last_laser;
ros::Time zero_time(0);
ros::Time laser_time(0);
sensor_msgs::LaserScan snapshot;
sensor_msgs::LaserScan last_snapshot;
ros::Time snapshot_time(0);

sensor_msgs::LaserScan subtractLaser(sensor_msgs::LaserScan laser){
//	ROS_INFO("Subtract laser using %f", snapshot_time.toSec());
        for(int i = 0; i < laser.ranges.size(); i++){
		//filter data outside of square arena
//		double y = laser.ranges[i]*cos(i*0.36);
//		double x = laser.ranges[i]*sin(i*0.36);
//		if(x < -2.0 || x > 2.0 || y < 0.0 || y > 2.0){
//			laser.ranges[i] = 10;
//		}else
		
		if(fabs(snapshot.ranges[i] - laser.ranges[i]) < 0.15){
                	laser.ranges[i] = 10.0; //set it above max range
                        //laser projection will ignore this value
                }else if(laser.ranges[i] > 3.0){
			//ignore far away values
		//	ROS_INFO("Ignoring FAR AWAY r=%f, theta=%f", laser.ranges[i], i*0.36);
			laser.ranges[i] = 10.0;
		}else{
			//do nothing, allow conversion
		//	ROS_INFO("Found theta=%f, r=%f, difference=%f", i*0.36, laser.ranges[i], fabs(snapshot.ranges[i] - laser.ranges[i]));	
		}

		
        }
        //after subtraction filter is finished, return result
        return laser;
}


//scanCallback - fires every time a laser scan is received
void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
	last_laser = *laser;
	laser_time = ros::Time::now();
	sensor_msgs::PointCloud2 cloud;
	if(snapshot_time.is_zero()){
		//if no snapshot taken, output laser directly
		projector_.projectLaser(*laser, cloud);
		output.publish(cloud);
	}else{
		//run subtraction filter before projection
		projector_.projectLaser(subtractLaser(*laser), cloud);	
		output.publish(cloud);
	}
}


void commandCallBack(const std_msgs::Int8::ConstPtr& command){
	if(command->data == 0){
		//use the last_laser as the snapshot for this command
		ROS_INFO("Taking snapshot");
		last_snapshot = snapshot;
		snapshot = last_laser;
		snapshot_time = ros::Time::now();	
	}else{
		//ignore the snapshot
		ROS_INFO("Removing snapshot");
		snapshot_time = zero_time;
	}
}



int main(int argc, char**argv){
	ros::init(argc, argv, "pcl_conversion_node");
	ros::NodeHandle nh;

	output = nh.advertise<sensor_msgs::PointCloud2>("lidar_pointcloud", 1);

	input = nh.subscribe("/scan", 1, scanCallBack);
	command = nh.subscribe("/command", 1, commandCallBack);	
		

	ros::spin();
	return 0;
}		
		
	
	
	
	
	

