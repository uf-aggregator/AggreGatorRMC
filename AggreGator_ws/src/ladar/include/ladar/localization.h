#ifndef _LOCALIZE_
#define _LOCALIZE_
#include "ladar_data.h"

class Localize {
	protected:
		float * Wall1, * Wall2, * Wall3;
		Ladar *ladarData;

	public:
		Localize();
		~Localize(){
			delete [] Wall1;
			delete [] Wall2;
			delete [] Wall3;
		}
		float min(std::vector<float> array) const;
		float* getWall(int number) const;
		void Localize::setWall(float distance, float angle, int WallNumber);
		float* unpolarize(std::vector<std::pair<float, float> > coordinates) const;
		float* adjustWall(float distance, float distancePrime, float angle, float anglePrime, int WallNumber);
		void update(std::vector<std::pair<float, float> > coordinates);
};
#endif