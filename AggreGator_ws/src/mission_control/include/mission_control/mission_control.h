#include "std_msgs/String.h"
#include "ros/ros.h"
#include "mission_control/state_handler.h"

#ifndef MISSION_CONTROL_H 
#define MISSION_CONTROL_H

class MissionControl {
	protected:
		StateHandler *sh;
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
		void Publish();
		void Subscribe();
		void Abort();
		void StartSenseAct();
		void StartPlanSenseAct();
};

#endif