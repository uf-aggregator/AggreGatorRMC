#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <vector>
#include "matrix.h"
#define AR 3
#define AC 3
#define BR 3
#define BC 1
#define CR 1
#define CC 3
#define DR 1
#define DC 1
#define XR 3
#define XC 1
#define YR 1
#define YC 1
#define UR 1
#define UC 1
#define ER 1
#define EC 1

using namespace std;

class SSController
{
	vector<vector<float> > A;
	vector<vector<float> > B;
	vector<vector<float> > C;
	vector<vector<float> > D;
	vector<vector<float> > X;
	vector<vector<float> > E; 
	vector<vector<float> > U;
	vector<vector<float> > Y;
 public:
	vector<vector<float> > update(); //updates the values of X, Y, U and returns new Y
	vector<vector<float> > getX(); //returns the X matrix
	vector<vector<float> > getY(); //returns the value of Y WITHOUT updating
	vector<vector<float> > getU();
	vector<vector<float> > getE();
	void setU(vector<vector<float> >); //sets the reference to be used for future updates
	void setU(float);
	SSController();
	SSController(vector<vector<float> >, vector<vector<float> >, vector<vector<float> >, vector<vector<float> >, vector<vector<float> >); //dimensions must be 3x3, 3x1, 1x3, 1x1;
	SSController(vector<vector<float> >); //dimensions must be 3x3, 3x1, 1x3, 1x1;
};
