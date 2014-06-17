#ifndef MISSION_CONTROL_H 
#define MISSION_CONTROL_H
#include <ros/ros.h>

class MissionControl {
	protected:
		ROS::NodeHandle nh
		ROS::Subscriber sub;
		ROS::Publisher pub;
		bool debug;

	public:
		MissionControl();
		MissionControl(bool debug);
		~MissionControl(){}
		void Abort();
		void Start();
		//void StateHandlerCallback();
};

#endif