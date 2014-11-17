#include "filter.h"

/***********************************************************************
	FUNCTION IMPLEMENTATIONS
**************************************************************************/


bool Filter::setOffset(sensor_msgs::LaserScan laser){
	offset.clear();
	
	for(int i = 0; i < laser.ranges.size(); i++){
		offset.push_back(laser.ranges[i]);
	}
	
	//set other ladar properties
	angle_min = laser.angle_min;
	angle_max = laser.angle_max;
	angle_increment = laser.angle_increment;
	range_min = laser.range_min;
	range_max = laser.range_max;
	num_samples = laser.ranges.size();
	
	offset_saved = 1; //indicate that an offset has been given
	
	return true;
}

bool Filter::setLastInput(sensor_msgs::LaserScan input){
	last_input.clear();
	for(int i = 0; i < num_samples; i++){
		last_input.push_back(input.ranges[i]);
	}
	
	input_saved = 1; //indicate that an input has been given
	
	return true;
}

bool Filter::isReady(){
	//indicate that the filter is ready
	return (offset_saved == 1) && (input_saved == 1); 
}

std::vector<float> Filter::runFilter(){
	filtered.clear();
	for(int i = 0; i < num_samples; i++){
		filtered.push_back(last_input[i] - offset[i]);
	}
	return filtered;
}
		
		
std::vector<float> Filter::runFilter(sensor_msgs::LaserScan input){
	setLastInput(input);
	return runFilter();
}


		
