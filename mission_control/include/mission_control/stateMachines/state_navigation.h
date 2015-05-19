#ifndef STATE_NAVIGATION_H
#define STATE_NAVIGATION_H

#include "../behaviors/behaviors.h"
#include "../behaviors/navigation_behavior.h"
#include "common_files/IRDistances.h"

class NavigationState {
	private:
		int obstaclesAt(bool left, bool right);

	public:
		NavigationState(States state);
		
		int navigateToMine();
		int navigateToDump();
		int navigateTo(bool mine);

		/* these should be moved to a behavior later */
		static common_files::IRDistances ir_distances;
		static void readIrDistancesCallback(const common_files::IRDistances::ConstPtr& msg);
		int checkIRs(bool front);
};

#endif