#include "ros/ros.h"
#include "hardware_interface/ina226.h"

int main(int argc, char** argv)
{
    //Initilize the ina226 node
    ros::init(argc, argv, "ina226_node");

    //Node handler this is how you work with ROS

    ros::NodeHandle n;
	
	//Set up publishers
	write_reg_pub = n.advertise<common_files::WriteI2CRegister>("write_i2c_register",1000);
	power_pub = n.advertise<common_files::ElectronicPowerData>("electronic_power",1000);
	
	//Set up service client
	read_register_srv = n.serviceClient<common_files::ReadI2CRegister>("read_i2c_register");
	
	while(write_reg_pub.getNumSubscribers()==0 && ros::ok()); //Wait until the publisher is fully connected to the subscriber
	
	//Initialize the INA226 chip
	INA226::inaInitialize();
	
    // Main loop
    while (ros::ok())
    {
		INA226::publishElectronicPower();
        ros::spinOnce();
    }
	
	return 0;
}
