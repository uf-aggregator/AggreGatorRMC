#include "ros/ros.h"

#include <vector>

#include <hardware_interface/i2c.h>


int main(int argc, char** argv)
{
	
    I2C *i2c = new I2C();
    //Initilize the i2c node
    ros::init(argc, argv, "test_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<common_files::ReadI2C>("read_i2c");


	//READING
        //prep the service parameters
        common_files::ReadI2C data;
        data.request.addr = 1;
        data.request.size = 2;



    while (ros::ok())
    {
        //fetch the value from the service
        if(client.call(data)){
		if(data.response.data[0] == 1){
			ROS_ERROR("Error reading i2c bus");
		}
               	for(int i = 0; i < data.response.data.size(); i++){
			ROS_INFO("%d", data.response.data[i]);
		}
        }else {
                ROS_ERROR("Failed to call the service read_i2c");
        }
	
        ros::spinOnce();
	ros::Duration(.1).sleep(); //sleep for a tenth of a second
    }
	
	return 0;
}
