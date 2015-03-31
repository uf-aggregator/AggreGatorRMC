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
#include "common_files/ReadLidar.h"

ros::Subscriber cluster_pc;
ros::Publisher centroid;
ros::ServiceServer read_srv;

float last_centroid[3] = {0.0, 0.0, 0.0};

void cluster_pcCallBack(const sensor_msgs::PointCloud2::ConstPtr& ros_pc_in){
        //convert ROS message into pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*ros_pc_in, *pc_in);
	
	//calculat centroid
	Eigen::Vector4f centroid;	
	pcl::compute3DCentroid(*pc_in, centroid);
	
	ROS_INFO("X:%f, Y:%f, Z:%f", centroid[0], centroid[1], centroid[2]);
	last_centroid[0] = centroid[0];
	last_centroid[1] = centroid[1];
	last_centroid[2] = centroid[2];
	
}

bool ReadLidarCallback(common_files::ReadLidar::Request&  request,
                     common_files::ReadLidar::Response& reply){
	reply.x = last_centroid[0];
	reply.y = last_centroid[1];
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pcl_centroid_node");
	ros::NodeHandle nh;

	cluster_pc = nh.subscribe<sensor_msgs::PointCloud2>("lidar_pointcloud", 1, cluster_pcCallBack);
//	output = nh.advertise<sensor_msgs::PointXYZ>("cluster_centroid", 1);
	
	read_srv = nh.advertiseService("read_lidar", ReadLidarCallback);


	while(ros::ok()){
		ros::spinOnce();
	}
}

