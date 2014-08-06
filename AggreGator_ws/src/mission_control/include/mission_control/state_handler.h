#ifndef STATE_HANDLER_H
#define STATE_HANDLER_H

#include <vector>
#include <queue>

enum states {

};

class StateHandler {
	protected:
		int current_state;
		int prev_state;
		std::vector<int> state_history;
		std::priority_queue<states> stateQueue;
		//intending to add more data members for decision-making
		//actionValue from ladar

	public:
		StateHandler();
		~StateHandler(){}

		int getCurrentState() const;
		int getPrevState() const;
		std::vector<int> getStateHistory() const;
		void setCurrentState(int newCurrentState);
		int nextState();
};

#endif