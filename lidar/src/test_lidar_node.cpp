#include "ros/ros.h"

#include <vector>

#include "common_files/ReadLidar.h"


int main(int argc, char** argv)
{
	
    ros::init(argc, argv, "test_lidar_node");

    ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<common_files::ReadLidar>("read_lidar");


	//READING
        //prep the service parameters
        common_files::ReadLidar service;



    while (ros::ok())
    {
        //fetch the value from the service
        if(client.call(service)){
		ROS_INFO("X: %f, Y:%f\n", service.response.x, service.response.y);
        }else {
                ROS_ERROR("Failed to call the service read_lidar");
        }
	
        ros::spinOnce();
	ros::Duration(.5).sleep(); //sleep for half a second
    }
	
	return 0;
}
