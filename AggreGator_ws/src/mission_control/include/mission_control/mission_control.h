#include "std_msgs/String.h"
#include "ros/ros.h"
#include <mission_control/ladar_handler.h>

#ifndef MISSION_CONTROL_H 
#define MISSION_CONTROL_H

class MissionControl {
	protected:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		LadarHandler *lh;
		bool debug;
		std::string class_name;

	public:
		MissionControl();
		MissionControl(bool debug);
		~MissionControl(){
			delete lh;
		}
		void Publish();
		void Subscribe();
		void Abort();
		void Start();
};

#endif