#include <iostream>
#include <vector>
#include "ladar/conversions.h"

using namespace std;

/*================================================
 *VECTOR -> ARRAY
 *================================================*/

/*-----------------------------------------	
 *	convertVectorToArray
 *		returns a 1d array that is twice vect's size
 *		in form [x1, y1, x2, y2, x3, y3, ... xn, yn]
 *-----------------------------------------*/
float* Conversions::convertVectorToArray(vector<pair <float, float> > vect) {
	float *result = new float[vect.size()*2];
	for(int i = 0, j = 0; i < vect.size() && j < vect.size(); i+=2, j++){
		result[i] = vect.at(j).first;
	}
	for(int i = 1, j = 0; i < vect.size()*2 && j < vect.size(); i+=2, j++){
		result[i] = vect.at(j).second;
	}
	return result;
}

/*-----------------------------------------	
 *	convertVectorToArray
 *		converts a float vector into a 1d float array and returns it
 *-----------------------------------------*/
float* Conversions::convertVectorToArray(vector<float> vect) {
	float *result = new float[vect.size()];
	for(int i = 0; i < vect.size(); i++){
		result[i] = vect.at(i);
	}
	return result;
}

/*-----------------------------------------	
 *	convertVectorToArray
 *		converts a int vector into a 1d int array and returns it
 *-----------------------------------------*/
int* Conversions::convertVectorToArray(vector<int> vect) {
	int *result = new int[vect.size()];
	for(int i = 0; i < vect.size(); i++){
		result[i] = vect.at(i);
	}
	return result;
}

/*================================================
 *ARRAY -> VECTOR
 *================================================*/

 /* -----------------------
  *	convertArrayToVector(float* array)
  *		Takes a float array, copies it a vector and returns a vector of type float	
  *------------------------*/
vector<float> Conversions::convertArrayToVector(float* array) {
	vector<float> vect(1);
	vect.clear();
	for(int i = 0; i < vect.size(); i++){
		vect.push_back(array[i]);
	}
	return vect;
}