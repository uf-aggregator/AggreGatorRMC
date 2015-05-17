#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <queue>
#include <string>

class StateMachine {
	public:
		//PUBLIC DATA MEMBERS==================
		//cap for how many states to retain
		static const int MAX_HISTORY = 100;

		//holds states that have been executed in descending order
		int stateHistory[MAX_HISTORY];
		int currentState, currentHistoryIndex;

		//constructors
		StateMachine();
		~StateMachine(){
		}

		//PUBLIC METHODS=======================
		/* int start(int)
		 *		master start function
		 */
		int start(int startState);

		/* int startStateMachine(int)
		 *		start the primary execution
		 */
		int startStateMachines(int startState);

		/* int executeState(int)
		 *		run a state
		 */
		int executeState(int executingState);

		/* int next(void)
		 *		decide next state
		 */
		int next();

		/* void printHistory(void)
		 *		prints out the state history in ascending order
		 */
		 void printHistory();

	private:
		static const int LOOP_LIMIT = 3;

		//PRIVATE METHODS=======================
		/* int check(int)
		 *		check function for preventing infinite loops
		 */
		int check(int value);
};

#endif
