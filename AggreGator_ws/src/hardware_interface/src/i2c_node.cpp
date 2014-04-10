#include "ros/ros.h"

#include <vector>

#include <i2c.h>
#include "hardware_interface/WriteI2C.h"
#include "hardware_interface/ReadI2C.h"
#include "hardware_interface/WriteI2CRegister.h"
#include "hardware_interface/ReadI2CRegister.h"

/*
 * Callbacks
 */
//Read from I2C
bool ReadI2CCallback(hardware_interface::ReadI2C::Request&  request,
                     hardware_interface::ReadI2C::Response& reply)
{
    char* buf;
    buf = read_i2c(request.addr, request.size);
    for(int i = 0; i < request.size; ++i)
    {
        reply.data.push_back(buf[i]);
    }
    free(buf);
    return true;
}

//Read from I2C register
bool ReadRegisterI2CCallback(hardware_interface::ReadI2CRegister::Request&  request,
                             hardware_interface::ReadI2CRegister::Response& reply)
{
    char* buf;
    buf = readfromreg_i2c(request.addr, request.reg, request.size);
    //Check dirty bit
    if(buf[0])  //Dirty bit set
    {
        free(buf);
        return false;
    }

    //Return with data
    for(int i = 1; i <= request.size; ++i)
    {
        reply.data.push_back(buf[i]);
    }

    free(buf);
    return true;
}

//Write to I2C
void WriteI2CCallback(const hardware_interface::WriteI2C& msg)
{
    char* data = (char*) malloc(msg.data.size());
    for(int i = 0; i < msg.data.size(); ++i)
    {
        data[i] = msg.data[i];
    }

    write_i2c(msg.addr, msg.data.size(), data);

    free(data);
}

//Write to I2C register
void WriteRegisterI2C(const hardware_interface::WriteI2CRegister& msg)
{
    char* data = (char*) malloc(msg.data.size());
    for(int i = 0; i < msg.data.size(); ++i)
    {
        data[i] = msg.data[i];
    }

    writetoreg_i2c(msg.addr, msg.reg, msg.data.size(), data);

    free(data);
}

/*
 * Main
 */
int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the i2c node
    ros::init(argc, argv, "i2c_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize the subscribers
    ros::Subscriber write_sub = n.subscribe("write_i2c", 1000, WriteI2CCallback);
    ros::Subscriber write_register_sub = n.subscribe("write_register_i2c", 1000, WriteRegisterI2C);

    //Initilize the services
    ros::ServiceServer read_srv = n.advertiseService("read_i2c", ReadI2CCallback);
    ros::ServiceServer read_register_srv = n.advertiseService("read_register_i2c", ReadRegisterI2CCallback);

    //Initilize the I2C bus
    init_i2c();


    /*
     * Main loop
     */
    while (ros::ok())
    {

        ros::spinOnce();
    }
}
