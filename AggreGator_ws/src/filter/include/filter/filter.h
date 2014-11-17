#ifndef _FILTER_H
#define _FILTER_H

#include <iostream>
#include <cstdlib>
#include <utility>
#include <vector>
#include <cmath>
#include <sstream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

class Filter{
	protected:
		//DATA MEMBERS
		std::vector<float> offset; //snapshot of initial room state
		std::vector<float> last_input; //current input
		int offset_saved;
		int input_saved;
		int num_samples;
		float angle_min, angle_max, angle_increment, range_min, range_max;
		std::vector<float> filtered;
		
	public:
		Filter(){
			offset_saved = 0;
			input_saved = 0;	
		}
		
		//GETTERS+SETTERS
		// Note: all angle, range, and num_sample properties are set with offset
		
		/*
		bool setOffset(sensor_msgs::LaserScan laser)
			-Sets the offset vector to be laser.ranges
			-Also sets all LADAR properties (range, angle, num_samples)
			-Should be run when environment changes
			
			-returns TRUE if success, false otherwise
		*/
		bool setOffset(sensor_msgs::LaserScan input);
		// setInput ONLY affects the last_input vector
		bool setLastInput(sensor_msgs::LaserScan input);
		
		float getAngleMin(){ return angle_min; }
		float getAngleMax(){ return angle_max; }
		float getAngleIncrement(){ return angle_increment; }
		float getRangeMin(){ return range_min; }
		float getRangeMax(){ return range_max; }
		int getNumSamples(){ return num_samples; }
		
		
		//FILTER FUNCTIONS
		
		/*
		bool runFilter()
			IF NO INPUT IS GIVEN:
			- Set the filtered vector to be last_input - offset 
			IF INPUT IS GIVEN:
			- Set the filtered vector to be input - offset
		*/
		std::vector<float> runFilter();
		std::vector<float> runFilter(sensor_msgs::LaserScan input); 
		
		std::vector<float> getFiltered(){ return filtered; } //returns filtered vector
		bool isReady();
};

#endif
