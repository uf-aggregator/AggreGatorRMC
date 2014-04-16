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

    vector<pair<float, float> > coordinates(numOfSamples);
    float theta;
    float x;
    float y;
    
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

  for(int i = 0; i < coordinate.size() - 1; i++){
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
void putpixel(SDL_Surface *surface, int x, int y, Uint32 pixel);

int Ladar::drawCoordinates()
{
 SDL_Surface *screen;
 int quit = 0;
 SDL_Event event;
 int x, y;
 Uint32 yellow;

 // Initialize defaults, Video and Audio
 if((SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO)==-1))
 {
  printf("Could not initialize SDL: %s.\n", SDL_GetError());
  return -1;
 }

 screen = SDL_SetVideoMode(800, 600, 24, SDL_SWSURFACE | SDL_FULLSCREEN);
 if ( screen == NULL )
 {
  fprintf(stderr, "Couldn't set 800x600x24 video mode: %s\n", SDL_GetError());
  return -2;
 }

 // Map the color yellow to this display (R=0xff, G=0xFF, B=0x00)
 yellow = SDL_MapRGB(screen->format, 0xff, 0xff, 0x00);
 
 // Make the dot at the center of the screen
 x = screen->w / 2;
 y = screen->h / 2;

 while( !quit )
 {
  // Poll for events
  while( SDL_PollEvent( &event ) )
  {
   switch( event.type )
   {
    case SDL_KEYUP:
     if(event.key.keysym.sym == SDLK_ESCAPE)
      quit = 1;
      break;
     if(event.key.keysym.sym == SDLK_F1)
      SDL_WM_ToggleFullScreen(screen); // Only on X11
      break;
    case SDL_QUIT:
     quit = 1;
     break;
    default:
     break;
   }
  }

  // Lock the screen for direct access to the pixels
  if ( SDL_MUSTLOCK(screen) )
  {
   if ( SDL_LockSurface(screen) < 0 )
   {
    fprintf(stderr, "Can't lock screen: %s\n", SDL_GetError());
    return -3;
   }
  }

  // Plot the Pixel
  putpixel(screen, x, y, yellow);

  // Unlock Surface if necessary
  if ( SDL_MUSTLOCK(screen) )
  {
   SDL_UnlockSurface(screen);
  }

  // Update just the part of the display that we've changed
  SDL_UpdateRect(screen, x, y, 1, 1);
 }

 SDL_Quit();
 
 return 0;
}//end drawCoordinates

void putpixel(SDL_Surface *surface, int x, int y, Uint32 pixel)
{
 int bpp = surface->format->BytesPerPixel;
 // Here p is the address to the pixel we want to set
 Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

 switch(bpp)
 {
  case 1:
   *p = pixel;
   break;
  case 2:
   *(Uint16 *)p = pixel;
   break;
  case 3:
   if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
   {
    p[0] = (pixel >> 16) & 0xff;
    p[1] = (pixel >> 8) & 0xff;
    p[2] = pixel & 0xff;
   } else {
    p[0] = pixel & 0xff;
    p[1] = (pixel >> 8) & 0xff;
    p[2] = (pixel >> 16) & 0xff;
   }
   break;
  case 4:
   *(Uint32 *)p = pixel;
   break;
 }
}//end putpixel
/*END GRAPHICS*/

//NOTE: Any value < 0.1 is like literally touching the ladar

bool Ladar::forwardCheck(){
    //make check for forward facing ladar
}

bool Ladar::leftCheck(){}
bool Ladar::rightCheck(){}

//potentially create functions for determining actual degrees, not in radians
//ladar scans counterclockwise, so mark with sticky note or something which way, during tests


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

