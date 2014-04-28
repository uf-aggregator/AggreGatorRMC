
#ifndef _LADAR_H_
#define _LADAR_H_
#include <vector>

class Ladar {
protected:
	std::vector<float> thetas, degrees, slopes;
	std::vector<std::pair<float, float> > coords;
	float angle_max, reference_angle;//max field of vision for the ladar
public:
	Ladar(){}
	Ladar(int numOfSamples);
	~Ladar(){}
	float adjustTheta(float theta);
	std::vector<std::pair<float, float> > getCoordinates(	float ranges[], int numOfSamples, 
												float angle_min, float angle_increment,
	                                            float min_range, float max_range);
	std::vector<std::pair<float, float> > fivePointAverager(std::vector<std::pair<float, float> > original);
	std::string coordinatesToString(std::vector<std::pair<float, float> > coordinates);
	std::vector<float> getThetas() const{
		return thetas;
	}
	std::vector<std::pair<float, float> > getCoordinates() const{
		return coords;
	}
	void setMaxAngle(float angle){
		this->angle_max = angle;
	}
	std::vector<float> getSlopes(std::vector<std::pair<float,float> > coordinates);
	std::vector<float> getAverageSlopes(std::vector<float> slopes);
	float getAverageSlope(std::vector<float> slopes);
	float getAverageSlope(std::vector<float> slopes, int startIndex, int endIndex);
	std::vector<int> findCorners(std::vector<float> slopes);
	void processSlopes();
	void print(std::vector<float> choice, std::string type);
	void print(int choice);
	void printDegreesOnly();
	void printSlopesOnly();
	void printRadiansOnly();
	int drawCoordinates(std::vector<std::pair<float, float> > coordinates);
	bool forwardCheck();
	bool leftCheck();
	bool rightCheck();
    
};

#endif
