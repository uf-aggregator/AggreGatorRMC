//
//  interpretRanges.cpp
//
//  Created by Joey Siracusa on 3/25/14.
//

#include <iostream>
#include <cstdlib>
#include <utility>
#include <vector>
#include <cmath>
#include <sstream>
#include "ladar/ladar_data.h"

using namespace std;

/*
	getCoordinates
		  -	Uses laser.ranges[] array, along with other laser member variables, 
			to convert the polar coordinates into cartesian coordinates	
		  -	Returns as vector of pairs.  First element of pair is x, second element is y.

*/
vector<pair<float, float> > getCoordinates(	float ranges[], int numOfSamples, 
											float angle_min, float angle_increment,
                                            float min_range, float max_range){
    
    vector<pair<float, float> > coordinates(numOfSamples);
    float theta;
    float x;
    float y;
    
    //converts index into theta based on angle_min and angle_increment
    //then converts polar (range, theta) into cartesian (x,y)
    
    
    for(int i = 0; i < numOfSamples; i++){
        theta = i*angle_increment + angle_min; //calculate theta assuming LADAR as origin
        if(ranges[i] > min_range && ranges[i] < max_range){
            //if the range meets the range constraints, push coordinates to vector
            x = ranges[i]*cos(theta); //calculate x coordinate
            y = ranges[i]*sin(theta); //calculate y coordinate
            pair<float, float> curr(x,y);
            coordinates.push_back(curr); //push to coordinates vector
            
        }
        //else, do not push coordinates to vector; they are not accurate
        
    }
    
    return coordinates;
}

/*
	fivePointAverager
		-Records every five values of x and y into currXs and currYs
		-On the fifth value, sorts currXs and currYs
		-Push the median of each list to our filtered list
*/

vector<pair<float, float> > fivePointAverager(vector<pair<float, float> > original){
    vector<pair<float, float> > filtered;
    float currXs[5];
    float currYs[5];
    int j = 0;
    
    for(int i = 0; i < original.size(); i++){
        currXs[j] = original[i].first; //record every five xs
        currYs[j] = original[i].second; //record every five ys
        j++;
        
        if(j == 5){
        	//after collecting five terms, do a bubble sort
			int temp, length;
			int length = 5; 
			bool swapped = true;
			//first sort X values
			while(swapped){
				swapped = false;
				//keep iterating through the list, until no values need swapping
    	   		for(int k = 0; k < length - 1; k++){
    	        	if(currXs[k] > currXs[k+1]){
						temp = currXs[k+1];
						currXs[k+1] = currXs[k];
						currXs[k] = temp;
                		swapped = true;
					}
				}
				
			}
			swapped = true;
			while(swapped){
				swapped = false;
				//keep iterating through the list, until no values need swapping
    	   		for(int k = 0; k < length - 1; k++){
    	        	if(currYs[k] > currYs[k+1]){
						temp = currYs[k+1];
						currYs[k+1] = currYs[k];
						currYs[k] = temp;
                		swapped = true;
					}
				}
				
			}
		
        	pair<float, float> curr(currXs[length/2], currYs[length/2]);
        	filtered.push_back(curr);
        	j = 0;
		}
        
        
    }
    
    return filtered;
}//end averager
}

/*
	coordinatesToString
		-Creates a string out of a vector of pairs.
		Format:
		
		"(x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) 
		 (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) "...

*/
string ghetto_to_string(float number){
    ostringstream buffer;
    buffer << number;
    return buffer.str();
}


string coordinatesToString(vector<pair<float, float> > coordinates){
    string coordString;
    for(int i = 0; i < coordinates.size(); i++){
        coordString +=  string("(") + ghetto_to_string(coordinates.at(i).first) + string(", ")
                                    + ghetto_to_string(coordinates.at(i).second) + string(") ");
        
        if((i+1)%10 == 0){
        	//only print 10 coordinates per line
        	coordString += string("\n");
        }
    }
    
    return coordString;
}

