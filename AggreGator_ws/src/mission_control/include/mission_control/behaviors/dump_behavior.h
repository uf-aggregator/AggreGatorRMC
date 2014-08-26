#ifndef DUMP_BEHAVIOR
#define DUMP_BEHAVIOR

#include "ros/ros.h"

class DumpBehavior : BaseBehavior {
	public:
		DumpBehavior();
		~DumpBehavior(){}
		std::string run();
	protected:
		void dump();
};

#endif