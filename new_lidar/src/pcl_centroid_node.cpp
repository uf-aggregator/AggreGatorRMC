/* locate_node
Node uses pcl_common's compute3DCentroid to get location of robot
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include "common_files/Centroid.h"
#include "common_files/Velocity.h"

ros::Subscriber cluster_pc;
ros::Publisher centroid_pub;
ros::Publisher velocity_pub;

float last_centroid[3] = {0.0, 0.0, 0.0};
float curr_centroid[3] = {0.0, 0.0, 0.0};
double last_centroid_time = 0.0;
double curr_centroid_time = 0.0;
int countSinceLastVel = 0; //only output velocity every two scans

void cluster_pcCallBack(const sensor_msgs::PointCloud2::ConstPtr& ros_pc_in){
        countSinceLastVel++;
	//convert ROS message into pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*ros_pc_in, *pc_in);
	
	//calculat centroid
	Eigen::Vector4f centroid;	
	pcl::compute3DCentroid(*pc_in, centroid);
	
	//ROS_INFO("PC height=%d, width=%d", pc_in->height, pc_in->width);	
	ROS_INFO("X:%f, Y:%f, Z:%f", centroid[1], centroid[0], centroid[2]);
	if(countSinceLastVel == 2){
		last_centroid[0] = curr_centroid[0];
		last_centroid[1] = curr_centroid[1];
		last_centroid[2] = curr_centroid[2];
		last_centroid_time = curr_centroid_time;
	}
	curr_centroid[0] = centroid[0];
	curr_centroid[1] = -centroid[1]; //x value needs to be negated for some reason
	curr_centroid[2] = centroid[2];
	curr_centroid_time = ros::Time::now().toSec();
	
	common_files::Centroid centroid_msg;
	centroid_msg.x = curr_centroid[1];
	centroid_msg.y = curr_centroid[0];
	centroid_pub.publish(centroid_msg);
/*
	float x_diff = curr_centroid[1] - last_centroid[1];
	float y_diff = curr_centroid[0] - last_centroid[0];	
	float time_diff = curr_centroid_time - last_centroid_time;
	if(last_centroid[0] != 0.0 && last_centroid[1] != 0.0 && last_centroid_time != 0.0 && countSinceLastVel == 2){
		common_files::Velocity velocity_msg;
		velocity_msg.x = x_diff/time_diff;
		velocity_msg.y = y_diff/time_diff;
		velocity_pub.publish(velocity_msg);
		ROS_INFO("x_diff: %f, y_diff: %f, time_diff: %f", x_diff, y_diff, time_diff);
	        ROS_INFO("Velocity: %f, %f", velocity_msg.x, velocity_msg.y);
		
		countSinceLastVel = 0;
	}
*/
}

//bool ReadLidarCallback(common_files::ReadLidar::Request&  request,
//                     common_files::ReadLidar::Response& reply){
//	reply.x = last_centroid[0];
//	reply.y = last_centroid[1];
//}

int main(int argc, char** argv){
	ros::init(argc, argv, "pcl_centroid_node");
	ros::NodeHandle nh;

	cluster_pc = nh.subscribe<sensor_msgs::PointCloud2>("lidar_pointcloud", 1, cluster_pcCallBack);
	centroid_pub = nh.advertise<common_files::Centroid>("centroid", 1);
	velocity_pub = nh.advertise<common_files::Velocity>("velocity", 1);
	
	
	while(ros::ok()){
		ros::spinOnce();
	}
}

