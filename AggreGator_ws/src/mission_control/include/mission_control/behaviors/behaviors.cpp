#include <iostream>
#include <ros/ros.h>
#include "behaviors.h"

void Behaviors::dump(){
	std::cout << "Dumping." << std::endl;
}

void Behaviors::mine(){
	std::cout << "Mining." << std::endl;
}

void Behaviors::move(){
	std::cout << "Moving." << std::endl;
}

void Behaviors::wait(){
	std::cout << "Waiting." << std::endl;
}