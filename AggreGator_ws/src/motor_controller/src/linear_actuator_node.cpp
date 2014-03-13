#include "ros/ros.h"
#include "std_msgs/builtin_int16.h"
#include "motor_controller/I2CMSG.h"

ros::Subscriber sub;
ros::Publisher pub;

motor_controller::I2CMSG GenerateMSG(int setpoint)
{
    motor_controller::I2CMSG msg;
    //Set msg addr
    //msg.addr = ???;           //Use an enumerated type from the I2C node

    //Set the msg data
    msg.data = setpoint;

    return msg;
}

motor_controller::I2CMSG LinearActuatorPID(int setpoint)
{
    /*
     * This is where the pid controller goes
     */

    return GenerateMSG(setpoint);               //Output the msg NOTE: change the passed parameter do not use setpoint directly
}

void SetSetpoint(const std_msgs::Int16& msg)
{
    pub.publish(LinearActuatorPID(msg.data));   //Generate and publish an I2C msg
}

//MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "linear_actuator_node");                              //Initialize node
    ros::NodeHandle n;                                                          //Create nodehandle object

    sub = n.subscribe("linear_actuator_rc", 1000, SetSetpoint);                 //Create object to subscribe to topic "linear_actuator_rc"
    pub = n.advertise<motor_controller::I2CMSG>("I2C",1000);                    //Create object to publish to topic "I2C"
	
    //Loop as long as ros system is ok
    ros::spin();

	
	return 0;
}
