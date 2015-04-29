#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#include "motor_utility.h"
#include "orientation_behavior.h"

enum States {
	START = 0,
	ORIENTATION_START,
	NAVIGATE,
	MINE,
	GO_HOME,
	ORIENTATION_DUMP,
	DUMP,
	GO_MINE
};

#endif
