#include <ros/ros.h>
#include <mission_control/ladar_handler.h>

using namespace std;
//====================================================
//CONSTRUCTORS
//====================================================
LadarHandler::LadarHandler(){
	class_name = "[LADAR HANDLER]";
}

//====================================================
//METHODS
//====================================================
void LadarHandler::executeActions(){
	cout << class_name << " running executeActions()" << endl;
}