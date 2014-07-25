#ifndef LADAR_HANDLER_H
#define LADAR_HANDLER_H

#include <ros/ros.h>
#include "mission_control/base_mid_handler.h"

class LadarHandler : public BaseMidLevelHandler {
	public:
		LadarHandler();
		~LadarHandler(){}
		void executeActions();
};

#endif