#ifndef _LOCALIZE_
#define _LOCALIZE_
#include <vector>
class Localize { 
	protected:
		float Wall1[2], Wall2[2], Wall3[2];

	public:
		Localize();
		~Localize(){
			delete [] Wall1;
			delete [] Wall2;
			delete [] Wall3;
		}
		float min(std::vector<float> array) const;
		float* getWall(int number);
		void setWall(float distance, float angle, int WallNumber);
		float polarize(std::pair<float, float> coordinates) const;
		void adjustWall(float distance, float distancePrime, float angle, float anglePrime, int WallNumber);
		void update(std::vector<std::pair<float, float> > coordinates);
};

#endif