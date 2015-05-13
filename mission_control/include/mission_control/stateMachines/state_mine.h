#ifndef STATE_MINE_H
#define STATE_MINE_H

#include "../behaviors/behaviors.h"
#include <ros/ros.h>

class MineState {
	public:
		MineState(States state);
		int mine();
};

#endif