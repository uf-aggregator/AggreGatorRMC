#ifndef STATE_DUMP_H
#define STATE_DUMP_H

#include "../behaviors/behaviors.h"
#include <ros/ros.h>

class DumpState {
	public:
		DumpState(States state);
		int dump();

};

#endif