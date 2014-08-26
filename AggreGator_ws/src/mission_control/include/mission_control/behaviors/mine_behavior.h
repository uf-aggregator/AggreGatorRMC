#ifndef MINE_BEHAVIOR
#define MINE_BEHAVIOR

#include "ros/ros.h"

class MineBehavior : BaseBehavior {
	public:
		MineBehavior();
		~MineBehavior(){}
		std::string run();
	protected:
		void mine();	
};

#endif