#ifndef NAVIGATION_BEHAVIOR_H
#define NAVIGATION_BEHAVIOR_H

class NavigationBehavior {
	public:
		static int moveStraight(bool front);
		static int turnLeft(bool front);
		static int turnRight(bool front);
		static int backUp(bool front);
};

#endif