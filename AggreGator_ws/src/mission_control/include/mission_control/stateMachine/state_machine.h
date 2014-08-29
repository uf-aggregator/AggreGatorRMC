#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <queue>
#include <string>



/*state
 *	stateId - id for stateHistory and deciding next state
 *	name - the name of the state, for logging purposes
 *	behavior - all the behaviors the state can start
 */
struct state {
	int stateId;
	int behaviorCount;
	int *behavior;
	std::string name;
};

class StateMachine {
	public:
	//DATA MEMBERS==================
		static const int MAX_HISTORY = 100;
		//all currently planned states
		state dump, wait, move, mine; 

		//holds behaviors to be executed
		std::queue<int> behaviorQueue;

		//holds states that have been executed in descending order
		int stateHistory[MAX_HISTORY];
		int currentState, currentHistoryIndex;

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

		/*void retrace(int numBack)
		 *	resets the current state to numBack states ago
		 */
		void retrace(int numBack);

		/*void printHistory(void)
		 *	prints out the state history in ascending order
		 */
		 void printHistory();
};

#endif