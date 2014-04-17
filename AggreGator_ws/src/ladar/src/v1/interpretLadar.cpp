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
#include "ladar/SDL/SDL.h"
#include "ladar/ladar_data.h"

using namespace std;

/*
    to_string not a function in roscpp, so using this because works
*/
string ghetto_to_string(float number){
    ostringstream buffer;
    buffer << number;
    return buffer.str();
}//end to_string

Ladar::Ladar(int numOfSamples): thetas(numOfSamples),
    degrees(numOfSamples),
    coords(numOfSamples){
    thetas.clear();
    degrees.clear();
    coords.clear();
}

/*
    getCoordinates
          - Uses laser.ranges[] array, along with other laser member variables, 
            to convert the polar coordinates into cartesian coordinates 
          - Returns as vector of pairs.  First element of pair is x, second element is y.

*/
vector<pair<float, float> > Ladar::getCoordinates(	float* ranges, int numOfSamples, 
											float angle_min, float angle_increment,
                                            float min_range, float max_range){
    //empty out current coordinates
    thetas.clear(); degrees.clear(); coords.clear();

    vector<pair<float, float> > coordinates(numOfSamples);
    float theta, x, y;
    
    //converts index into theta based on angle_min and angle_increment
    //then converts polar (range, theta) into cartesian (x,y)
    
    
    for(int i = 0; i < numOfSamples; i++){
        theta = i*angle_increment + angle_min;
        this->degrees.push_back(theta*57.296);
         //calculate theta assuming LADAR as origin
        if(ranges[i] > min_range && ranges[i] < max_range){
            //if the range meets the range constraints, push coordinates to vector
            x = ranges[i]*cos(theta); //calculate x coordinate
            y = ranges[i]*sin(theta); //calculate y coordinate
            pair<float, float> curr(x,y);
            coordinates.push_back(curr); //push to coordinates vector
            this->coords.push_back(curr);
            this->thetas.push_back(theta);
        }
        //else, do not push coordinates to vector; they are not accurate
        
    }

    return coordinates;
}//end getcoordinates

/*
	fivePointAverager
		-Adds together every five x and y values into currXSum and currYSum
		-On the fifth value, divide both currXSum and currYSum by 5
		-Push this averaged pair onto the filtered vector, reset the sums
*/

vector<pair<float, float> > Ladar::fivePointAverager(vector<pair<float, float> > original){
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
			length = 5; 
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

/*
	coordinatesToString
		-Creates a string out of a vector of pairs.
		Format:	
		"(x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) 
		 (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) (x, y) "...
*/

string Ladar::coordinatesToString(vector<pair<float, float> > coordinates){
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
}//end coordinatestostring

vector<float> Ladar::getSlopes(vector<pair<float,float> > coordinates){
  vector<float> slopes(coordinates.size()-1);
  slopes.clear();

  for(int i = 0; i < coordinates.size() - 1; i++){
     float x1 = coordinates.at(i).first, y1 =  coordinates.at(i).second;
     float x2 = coordinates.at(i+1).first, y2 =  coordinates.at(i+1).second;
     float slope = (y2-y1)/(x2-x1);
     slopes.push_back(slope);
  }

  return slopes;
}//end getSlopes

vector<float> Ladar::getAverageSlopes(vector<float> slopes){}

//change return to walls values I guess
void Ladar::processSlopes(){}


/*Prints out 
*/
void Ladar::print(vector<float> choice, string type){
    for(int i = 0; i < choice.size() && i < coords.size(); i++){
        string coord = string("(") + ghetto_to_string(coords.at(i).first) + string(", ")
                                    + ghetto_to_string(coords.at(i).second) + string(") m,m");
        cout << choice.at(i) << " " << type << " :  " << coord << endl;
    }
}

void Ladar::print(int choice){
    switch(choice){
        case 0:{
            print(this->thetas, "radians");
            break;
        }
        case 1:{
            print(this->degrees, "degrees");
            break;
        }
        default: break;
    }
}

/*GRAPHICS RENDERING*/
int Ladar::drawCoordinates(vector<pair<float, float> > coordinates)
{
}
/*END GRAPHICS*/


//NOTE: Any value < 0.1 is like literally touching the ladar

bool Ladar::forwardCheck(){
    //make check for forward facing ladar
}
bool Ladar::leftCheck(){}
bool Ladar::rightCheck(){}

/*
	Function for finding corners from vector of slopes
*/
vector<int> Ladar::findCorners(vector<float> slopes){
    vector<int> corners;
    float currSum = 0;
    float currAvg = 0;
    int currCount = 0;
    for(int i = 0; i < slopes.size() - 1; i++){
        currSum += slopes.at(i);
        currCount++;
        currAvg = currSum/((float)currCount);
        
        //if the next slope is off by the currAvg by more than 20%, assume this is a corner
        if(abs(slopes.at(i+1)) > abs(currAvg*1.25) || abs(slopes.at(i+1)) < abs(currAvg*.75)){
            corners.push_back(i+1);
            currSum = 0;
            currAvg = 0;
            currCount = 0;
        }
    
    }
    return corners;
}

