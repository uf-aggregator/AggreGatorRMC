#include <iostream>
#include <ros/ros.h>
#include "behaviors.h"
#include "unistd.h"

void Behaviors::dump(){
	usleep(1000000);
	std::cout << "Dumping." << std::endl;
}

void Behaviors::mine(){
	std::cout << "Mining." << std::endl;
}

void Behaviors::move(){
	usleep(1000000);
	std::cout << "Moving." << std::endl;
}

void Behaviors::wait(){
	std::cout << "Waiting." << std::endl;
}