#include "state_navigation.h"

NavigationState::NavigationState(States state){
	//for initializing data members for the particular state
	switch(state){
		case NAVIGATE:
			std::cout << "NAVIGATE" << std::endl;
			break;

		case GO_HOME:
			std::cout << "GO_HOME" << std::endl;
			break;

		case GO_MINE:
			std::cout << "GO_MINE" << std::endl;
			break; 
	}
}

int NavigationState::navigateToDump() {
	return 0;
}

int NavigationState::navigateToMine() {
	return 0;
}
