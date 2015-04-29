#include "state_mine.h"

MineState::MineState(States state){
	switch(state){
		case MINE:
			std::cout << "MINE" << std::endl;
			break;
	}
}