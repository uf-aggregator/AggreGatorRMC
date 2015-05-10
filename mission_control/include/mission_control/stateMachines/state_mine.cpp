#include "state_mine.h"

MineState::MineState(States state){
	//for initializing data members for the particular state
	switch(state){
		case MINE:
			std::cout << "MINE" << std::endl;
			break;
	}
}

int MineState::mine() {
	return 0;
}