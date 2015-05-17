#mission_control_node (MCN)
-----

This node is intended to contain all the state machines and execute the autonomy instructions.

It pulls in data from other packages and uses that in its decision-making.


###State Machine Diagram

Currently, this is the state machine we've implemented for the purpose of mining at the competition.

![State Machine Diagram](http://drive.google.com/uc?export=view&id=0B1M-wtPPPk3vUE0zUzJMLW1rYjA)

Each state is represented or consolidated in a `*_state.cpp` file in the `include/` folder here.