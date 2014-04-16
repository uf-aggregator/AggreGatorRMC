#ifndef _ladar_
#define _ladar_

class Ladar {
protected:
	std::vector<float> thetas;
	std::vector<std::pair<float, float> > coords;
	std::vector<float> degrees;
			
public:
	Ladar(){}
	Ladar(int numOfSamples);
	~Ladar(){}
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
	std::vector<float> getSlopes(std::vector<std::pair<float,float> > coordinates);
	std::vector<float> getAverageSlopes(std::vector<float> slopes);
	void processSlopes();
	void print(std::vector<float> choice, std::string type);
	void print(int choice);
	int drawCoordinates();
	bool forwardCheck();
	bool leftCheck();
	bool rightCheck();
    std::vector<int> findCorners(std::vector<float> slopes);
};

#endif
