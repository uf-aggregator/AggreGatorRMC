#ifndef MISSION_CONTROL_H 
#define MISSION_CONTROL_H

#include "std_msgs/String.h"
#include "ros/ros.h"
#include "stateMachine/state_machine.h"

class MissionControl {
	protected:
		bool debug;
		std::string class_name;

		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;

	public:
		MissionControl();
		MissionControl(bool debug);
		~MissionControl(){
		}
		void publish();
		void subscribe();
		void abort();
		void start();
};

#endif