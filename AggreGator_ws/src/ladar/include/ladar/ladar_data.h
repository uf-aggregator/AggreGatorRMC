#ifndef _ladar_
#define _ladar_

std::vector<std::pair<float, float> > getCoordinates(	float ranges[], int numOfSamples, 
											float angle_min, float angle_increment,
                                            float min_range, float max_range);

std::vector<std::pair<float, float> > fivePointAverager(std::vector<std::pair<float, float> > original);

std::string coordinatesToString(std::vector<std::pair<float, float> > coordinates);


#endif
