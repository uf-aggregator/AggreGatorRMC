#include "ros/ros.h"
//#include "hardware_interface/ADC.h"

int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the adc node
    ros::init(argc, argv, "adc_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize the ADCs
    //InitADC();

    /*
     * Main loop
     */
    while (ros::ok())
    {
        ros::spinOnce();
    }
}
