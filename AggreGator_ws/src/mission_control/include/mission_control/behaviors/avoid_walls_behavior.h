#ifndef MOVE_BEHAVIOR_H
#define MOVE_BEHAVIOR_H

#include "base_behavior.h"

class AvoidWallsBehavior: public BaseBehavior {
	public:
		AvoidWallsBehavior();
		~AvoidWallsBehavior(){}
		std::string run();

	protected:
		void avoid();
};

#endif