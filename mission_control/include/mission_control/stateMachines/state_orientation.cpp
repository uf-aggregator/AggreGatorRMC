#include "state_orientation.h"

OrientationState::OrientationState(States state){
	switch(state){
		case ORIENTATION_START:
			std::cout << "ORIENTATION_START" << std::endl;
			break;

		case ORIENTATION_DUMP:
			std::cout << "ORIENTATION_DUMP" << std::endl;
			break;
	}
}