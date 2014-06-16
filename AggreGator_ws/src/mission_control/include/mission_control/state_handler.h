#ifndef STATE_HANDLER_H
#define STATE_HANDLER_H

class StateHandler {
	protected:
		int current_state;
		int prev_state;

	public:
		StateHandler();
		~StateHandler(){}

		int getCurrentState() const;
		int getPrevState() const;
		void setCurrentState(int newCurrentState);
		
};

#endif