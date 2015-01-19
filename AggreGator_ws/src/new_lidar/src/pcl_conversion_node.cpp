//PCL Converter
//This node takes sensor_msgs/LaserScan from the urg_node and converts them into PointCloud2 messages
//The class comes with includes a pre-filter function for filtering before the PCL conversion

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/time.h>
#include <ros/duration.h>

laser_geometry::LaserProjection projector_; //used for the conversion
ros::Publisher output; //output for point clouds
ros::Subscriber input; //input for laser scans


//scanCallback - fires every time a laser scan is received
void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
	sensor_msgs::PointCloud2 cloud;
	projector_.projectLaser(*laser, cloud);
	output.publish(cloud);
}



int main(int argc, char**argv){
	ros::init(argc, argv, "pcl_conversion_node");
	ros::NodeHandle nh;

	output = nh.advertise<sensor_msgs::PointCloud2>("lidar_pointcloud", 1);

	input = nh.subscribe("/scan", 1, scanCallBack);
	
		

	while(ros::ok()){
		ros::spinOnce();
	}
	return 0;
}		
		
	
	
	
	
	

