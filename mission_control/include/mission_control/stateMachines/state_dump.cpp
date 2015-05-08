#include "state_dump.h"

DumpState::DumpState(States state){
	switch(state){
		case DUMP:
			std::cout << "DUMP" << std::endl;
			break;
	}
}