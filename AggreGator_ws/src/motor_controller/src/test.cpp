#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include "matrix.h"
#include "controller.h"

using namespace std;

int main (int argc, char* argv[])
{
	SSController wheel1; 

	//input
	vector<vector<float> > U;
 	U.push_back(vector<float> (1, 1.0));

	if(argc > 1)
		U[0][0] = atof(argv[1]);

	wheel1.setU(U);

	int iterations = 1500;
	if (argc > 2)
		iterations = atoi(argv[2]);

	for(int i = 0; i < iterations; i++)
	{
		wheel1.update();
		cout << "U: " << wheel1.getU()[0][0] << " Y: " << wheel1.getY()[0][0] << " E: " << wheel1.getE()[0][0] << endl;
		cout << "X: ";
		matrix_print(wheel1.getX());
		cout << endl;
		usleep(10000);
		if (i%100==0) //every hundred iterations prompt for an input update, so you can see how the controller reacts
		{
			float you;
			cin >> you;
			U[0][0] =you;
			wheel1.setU(U);
		}
	}

	return 0;
}
