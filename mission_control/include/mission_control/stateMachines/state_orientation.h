#ifndef STATE_ORIENTATION_H
#define STATE_ORIENTATION_H

#include "../behaviors/behaviors.h"

class OrientationState {
	public:
		OrientationState(States state);
		int orientToMine();
		int orientToDump();
};

#endif