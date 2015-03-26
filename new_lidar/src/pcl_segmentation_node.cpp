//segmentation_node
//uses PCL segmentation library to break pointcloud into euclidian segments

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Subscriber input;
ros::Publisher output;

void segmentCallBack(const sensor_msgs::PointCloud2::ConstPtr& ros_pc_in){
	
	//convert ROS message into pcl::PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*ros_pc_in, *pc_in);

	//create KdTree to search for clusters, using pc_in as input cloud
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pc_in);

	//create vector to remember indices of clusters	
	std::vector<pcl::PointIndices> cluster_indices; 
	
	//create EuclideanClusterExtraction object
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	
	//set EC configurations
	ec.setClusterTolerance(0.01); //cluster tolerance estimated 1 cm
	ec.setMinClusterSize(2); //guesstimation
	ec.setMaxClusterSize(2147483647); //guesstimation
	ec.setSearchMethod(tree); //use the KdTree created above
	ec.setInputCloud(pc_in); //use pc_in as input cloud
	
	ec.extract(cluster_indices); //perform the extraction
	
	//now grab the points with these indices out of the original pc_in cloud

	sensor_msgs::PointCloud2::Ptr ros_pc_out (new sensor_msgs::PointCloud2);
	
	for(std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin(); 
		cluster_it != cluster_indices.end(); ++cluster_it){
		//create new cloud for this cluster
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster( new pcl::PointCloud<pcl::PointXYZ>);	
		//copy ONLY relevant indices into cloud
		pcl::copyPointCloud(*pc_in, cluster_it->indices, *cloud_cluster);
		//convert it back to a ros message
		pcl::toROSMsg(*cloud_cluster, *ros_pc_out);

		output.publish(ros_pc_out);
	}	
		
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pcl_segmentation_node");
	ros::NodeHandle nh;

	input = nh.subscribe<sensor_msgs::PointCloud2>("lidar_pointcloud", 1, segmentCallBack);
	output = nh.advertise<sensor_msgs::PointCloud2>("cluster_pointcloud", 1);

	while(ros::ok()){
		ros::spinOnce();
	}


}
