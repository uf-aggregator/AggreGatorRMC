#include "state_start.h"

StartState::StartState(States state){
	switch(state){
		case START:
			std::cout << "START" << std::endl;
	}
}