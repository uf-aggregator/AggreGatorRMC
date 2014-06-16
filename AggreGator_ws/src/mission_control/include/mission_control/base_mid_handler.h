#ifndef BASE_MID_HANDLER_H
#define BASE_MID_HANDLER_H
#include <ros/ros.h>

class BaseMidLevelHandler {
	protected:
		int state_id;
	public:
		ROS::Subscriber sub;
		ROS::Publisher pub;
		ROS::NodeHandle nh;

		BaseMidLevelHandler();
		~BaseMidLevelHandler(){}

		void executeActions();
		int getStateId() const;
		void setStateId(int newId) const;
};

#endif BASE_MID_HANDLER_H