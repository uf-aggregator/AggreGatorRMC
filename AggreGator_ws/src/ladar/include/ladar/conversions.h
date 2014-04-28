#ifndef _CONVERSIONS_H_
#define _CONVERSIONS_H_
#include <vector>

class Conversions {
	protected:
	public:
		Conversions(){}
		~Conversions(){}
		float* convertVectorToArray(std::vector<std::pair <float, float> > vect);
		float* convertVectorToArray(std::vector<float> vect);
		int* convertVectorToArray(std::vector<int> vect);
		std::vector<float> convertArrayToVector(float *array);
};

#endif