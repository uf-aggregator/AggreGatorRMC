#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <queue>
#include <string>

class StateMachine {
	public:
	//DATA MEMBERS==================
		//cap for how many states to retain
		static const int MAX_HISTORY = 100;

		//holds states that have been executed in descending order
		int stateHistory[MAX_HISTORY];
		int currentState, currentHistoryIndex;

		//constructors
		StateMachine();
		~StateMachine(){
		}

	//METHODS=======================
		/*int start(int)
		 *	start from a certain state
		 */
		int start(int startingQueue);

		/*int next(void)
		 *	decide next state
		 */
		int next();

		/*void printHistory(void)
		 *	prints out the state history in ascending order
		 */
		 void printHistory();

		 //CALLBACKS
};

#endif
