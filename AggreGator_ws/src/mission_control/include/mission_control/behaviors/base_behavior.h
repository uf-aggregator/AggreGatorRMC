#ifndef BEHAVIOR_TEMPLATE_H
#define BEHAVIOR_TEMPLATE_H

#include <string>

class BaseBehavior {
	protected:
		//Data Members
		bool on;
		std::string name;
	public:
		BaseBehavior();
		~BaseBehavior(){}
		//Methods
		virtual std::string run() = 0;
		
};

#endif