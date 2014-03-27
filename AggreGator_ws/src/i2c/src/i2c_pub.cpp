#include "ros/ros.h"
#include "std_msgs/String.h"
#include "i2c/ina226.h"

//compare command line args for equality
bool equal(char *one, char *two){
	int lengthOne = one.length, lengthTwo = two.length;
	if(lengthOne != lengthTwo) return false;
	
	int length = one.length;
	
	for(int i = 0; i < length; i++){
		if(one[i] == two[i]) continue;
		else return false;
	}//endfor
	
	return true;
}

//map node name to number
int determineNode(char *nodeName){
	//make i2c calls here
}

int main(int argc, char **argv){
	//variables
	int node_number;
	
	//initiate ros and give name to node
	//create nodehandler object
	ros::init(argc, argv, "i2c_pub_node");
	ros::NodeHandle n;
	
	//will be instantiated in the loop
	ros::Publisher pub;
	ros::Rate loop_rate(10);
	
	//determine the topic to publish to
	node_number = determineNode();
	
	while(ros::ok()){
		//handle switch() statement of what's receiving info
		switch(node_number){
			case 0:
				pub = n.advertise<>("", 1000);
				break;
			case 1:
				pub = n.advertise<>("", 1000);
				break;
			default:
				pub = n.advertise<>("", 1000);
				break;			
			}//end switch
	}//end while
}
