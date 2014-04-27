#include <iostream>
#include <math>
#include <limits>
#include "ladar/localization.h"

using namespace std;

Localize::Localize() {
	Wall1 = {0, 0};
	Wall2 = {0, 0};
	Wall3 = {0, 0};
	ladarData = new Ladar();
}

float Localize::min(vector<float> array){
	//ignores cases where array[i] = 0
	int check = 0;
	float min = array.at(check);

	while(min == 0.0 && check < size) {
		min = array.at(check);
		check++;
	}//end while
	for(int i = 0; i < array.size(); i++){
		if(min > array.at(i)) min = array.at(i);
	}//end for
	return min;
}//end min

float* Localize::unpolarize(<pair<float, float> > coordinates) const {
	//will return unpolarized coordinates
}//end unpolarize

float* Localize::getWall(int number) const {
	switch(number){
		case 1: return Wall1;
		case 2: return Wall2;
		case 3: return Wall3;
		default: {
			cout << "GETWALL: Not a valid selection." << endl;
			return 0.0;
		}
	}
}//end getWall

void Localize::setWall(float distance, float angle, int WallNumber){
		switch(WallNumber){
			case 1: {
				Wall1[0] = distance;
				Wall1[1] = angle;
				break;
			}
			case 2: {
				Wall2[0] = distance;
				Wall2[1] = angle;
				break;
			}
			case 3: {
				Wall3[0] = distance;
				Wall3[1] = angle;
				break;
			}
			default:{
				cout << "SETWALL, switch: No such wall."<< endl;
				break;
			}
		}//end switch
}
void Localize::adjustWall(float distance, float distancePrime, float angle, float anglePrime, int WallNumber) {
	/*storeTempDist[#] 		=>	distance
	 *storeTempAngle[#]		=>	angle
	 *Wall#[0]				=>	distancePrime
	 *Wall#[1]				=>	anglePrime
	 */
	if(getWall(WallNumber)[] == 0 || getWall(WallNumber)[] == 0) setWall(distance, angle, WallNumber);
	else {	
		if(abs(distance/distancePrime) < 0.05 && abs(angle/anglePrime) < 0.05){
			setWall(distance, angle, WallNumber);
		}//endif
	}//end else
	cout <<"ADJUSTWALL: Could not satisfy conditions. Here's some junk."<< endl;
}//end adjustWall


void Localize::update(vector<pair<float, float> > coordinates) {
	//temporary data members
	int size = coordinates.size()/5, offset2, offset3, decisionMaker;
	vector<pair<float, float> > averagedPoints;
	vector<float> noiselessSlopes, storeTempWall1(size), storeTempWall2(size),
								storeTempWall3(size);
	float absurd;
	float storeTempDist[3], storeTempAngle[3], unpolar[coordinates.size()]; 

	//init some of the temporary data members
	offset2 = 0;
	offset3 = 0;
	absurd = numeric_limits<float>::max();

	for(int i = 0; i < 3; i++){
		storeTempDist[i] = absurd;
	}
	for(int i = 0; i < 3; i++){
		storeTempAngle[i] = absurd;
	}

	storeTempWall1.clear();
	storeTempWall2.clear();
	storeTempWall3.clear();

	//To be processed
	averagedPoints = ladarData->fivePointAverager(coordinates);
	noiselessSlopes = ladarData->getSlopes(averagedPoints);
		//unpolarize
		for(int i = 0; i < coordinates.size(); i++){
			//unpolar[i] = unpolarize(coordinates.at(i));
		}

	//algo starts here****************************************************
		decisionMaker = 1
		for(int i = 1; i < size; i++){
			if(abs(noiselessSlopes.at(i)/noiselessSlopes.at(i-1)) < 0.05){
				if(decisionMaker == 1) storeTempWall1.push_back(noiselessSlopes.at(i-1));
				else if(decisionMaker == 2) storeTempWall2.push_back(noiselessSlopes.at(i-1));
				else if(decisionMaker == 3) storeTempWall3.push_back(noiselessSlopes.at(i-1));
			}//endif
			else {
				decisionMaker++;
				if(decisionMaker == 2) offset2 = i;
				else if(decisionMaker == 3) offset3 = i;

				if(decisionMaker == 2) storeTempWall2.push_back(noiselessSlopes.at(i-1));
				if(decisionMaker == 3) storeTempWall3.push_back(noiselessSlopes.at(i-1));
			}//end else
		}//endfor

		//float getAverageSlope(slopes, startIndex, endIndex)		it will include the endIndex
		//float getAverageSlope(slopes)								alt method
		switch(decisionMaker){
			case 1: {
				storeTempDist[decisionMaker] = min(storeTempWall1);
				storeTempAngle[decisionMaker] = atan(ladarData->getAverageSlope(noiselessSlopes, 0, offset2 - 1)) * 180/PI;
				break;
			}
			case 2: {
				storeTempDist[decisionMaker] = min(storeTempWall2);
				storeTempAngle[decisionMaker] = atan(ladarData->getAverageSlope(noiselessSlopes, offset2, offset3 - 1)) * 180/PI;;
				break;
			}
			case 3: {
				storeTempDist[decisionMaker] = min(storeTempWall3);
				storeTempAngle[decisionMaker] = atan(ladarData->getAverageSlope(noiselessSlopes, offset3 - size - 1)) * 180/PI;;
				break;
			}
			default: break;
		}

		//set Wall# values
			//Wall1
			adjustWall(storeTempDist[0], Wall1[0], storeTempAngle[0], Wall1[1], 1);
			adjustWall(storeTempDist[1], Wall1[0], storeTempAngle[1], Wall1[1], 1);
			adjustWall(storeTempDist[2], Wall1[0], storeTempAngle[2], Wall1[1], 1);

			//Wall2
			adjustWall(storeTempDist[0], Wall2[0], storeTempAngle[0], Wall2[1], 2);
			adjustWall(storeTempDist[1], Wall2[0], storeTempAngle[1], Wall2[1], 2);
			adjustWall(storeTempDist[2], Wall2[0], storeTempAngle[2], Wall2[1], 2);

			//Wall3
			adjustWall(storeTempDist[0], Wall3[0], storeTempAngle[0], Wall3[1], 3);
			adjustWall(storeTempDist[1], Wall3[0], storeTempAngle[1], Wall3[1], 3);
			adjustWall(storeTempDist[2], Wall3[0], storeTempAngle[2], Wall3[1], 3);
	//algo ends here************************************************
}//end update