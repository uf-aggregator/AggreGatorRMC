#ifndef BASE_MID_HANDLER_H
#define BASE_MID_HANDLER_H

#include <iostream>
#include <ros/ros.h>

class BaseMidLevelHandler {
	protected:
		std::string class_name;
		int state_id; //id of the handler

		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;

	public:
		BaseMidLevelHandler();
		~BaseMidLevelHandler(){}
		virtual void executeActions() = 0; //must be overriden by extending class
		int getStateId() const;
		void setStateId(int newId);
};

#endif