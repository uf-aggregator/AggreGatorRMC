#include "ros/ros.h"

#include <vector>

#include <i2c.h>

int main(int argc, char** argv)
{
    I2C *i2c = new I2C();
    //Initilize the i2c node
    ros::init(argc, argv, "i2c_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize the subscribers
    ros::Subscriber write_sub = n.subscribe("write_i2c", 1000, I2C::WriteI2CCallback);
    ros::Subscriber write_register_sub = n.subscribe("write_i2c_register", 1000, I2C::WriteRegisterI2C);

    //Initilize the services
    ros::ServiceServer read_srv = n.advertiseService("read_i2c", I2C::ReadI2CCallback);
    ros::ServiceServer read_register_srv = n.advertiseService("read_i2c_register", I2C::ReadRegisterI2CCallback);

    //Initilize the I2C bus
    i2c->init_i2c();

    while (ros::ok())
    {
        ros::spinOnce();
    }
	
	return 0;
}
