#ifndef BASE_MID_HANDLER_H
#define BASE_MID_HANDLER_H

#include <iostream>

class BaseMidLevelHandler {
	protected:
		std::string class_name;
		int state_id;

	public:
		BaseMidLevelHandler();
		~BaseMidLevelHandler(){}

		virtual void executeActions() = 0;
		int getStateId() const;
		void setStateId(int newId);
};

#endif