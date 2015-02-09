#include "ros/ros.h"
#include "hardware_interface/AdaFruit.h"

int main(int argc, char** argv)
{    
    AdaFruit *af;

    //Initilize ros
    ros::init(argc, argv, "adafruit_node");

    int runtimeResult = af->run();

    return 0;
}
