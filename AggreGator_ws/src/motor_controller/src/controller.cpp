#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include "controller.h"

using namespace std;

SSController::SSController()
{	

/*
	float A_def[AR][AC] = 
	{
		{1.0000, 0.2620, 0},
		{0,    0.2275,    0.0017},
		{0,   -0.0397,   -0.0003}
	};

	float B_def[BR][BC] =
	{
		{0},
		{1.0279},
		{0.0528}
	};

	float C_def[CR][CC] = 
	{
		{.0721, .0152, 0}
	};

	float D_def[DR][DC] = {{0.0049}};
	*/



	float A_def[AR][AC] = 
	{
		{1.0000, 0.4772, 0},
		{0,    0.0000,    0.0000},
		{0,   -0.000,   -0.0000}
	};

	float B_def[BR][BC] =
	{
		{0},
		{ 1.4611}, 
		{0.0}
	};

	float C_def[CR][CC] = 
	{
		{.5903, .2113, 0}
	};

	float D_def[DR][DC] = {{0.1029}};

	float X_def[XR][XC] = {{0}};

	float Y_def[YR][YC] = {{0}};

	float U_def[UR][UC] = {{1}};

	for (unsigned int i = 0; i < AR; i++)
		A.push_back(vector<float> (A_def[i], A_def[i]+sizeof(A_def[i])/sizeof(float)));
	
	for (unsigned int i = 0; i < BR; i++)
		B.push_back(vector<float> (B_def[i], B_def[i]+sizeof(B_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < CR; i++)
		C.push_back(vector<float> (C_def[i], C_def[i]+sizeof(C_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < DR; i++)
		D.push_back(vector<float> (D_def[i], D_def[i]+sizeof(D_def[i])/sizeof(float)));
	
	for (unsigned int i = 0; i < XR; i++)
		X.push_back(vector<float> (X_def[i], X_def[i]+sizeof(X_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < YR; i++)
		Y.push_back(vector<float> (Y_def[i], Y_def[i]+sizeof(Y_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < UR; i++)
		U.push_back(vector<float> (U_def[i], U_def[i]+sizeof(U_def[i])/sizeof(float)));
	
	E=matrix_sub(U, Y);

/*	matrix_print(A);
	cout << endl;
	matrix_print(B);
	cout << endl;
	matrix_print(C);
	cout << endl;
	matrix_print(D);
	cout << endl;
	matrix_print(X);
	cout << endl;
	matrix_print(Y);
	cout << endl;
	matrix_print(U);
	cout << endl;
	matrix_print(E);
	cout << endl;*/
}

SSController::SSController(vector<vector<float> > U_def)
{
	
	float A_def[AR][AC] = 
	{
		{1.0000, 0.4772, 0},
		{0,    0.0000,    0.0000},
		{0,   -0.000,   -0.0000}
	};

	float B_def[BR][BC] =
	{
		{0},
		{ 1.4611}, 
		{0.0}
	};

	float C_def[CR][CC] = 
	{
		{.5903, .2113, 0}
	};


	float D_def[DR][DC] = {{0.1029}};


	float X_def[XR][XC] = {{0}};

	float Y_def[YR][YC] = {{0}};
	

	for (unsigned int i = 0; i < AR; i++)
		A.push_back(vector<float> (A_def[i], A_def[i]+sizeof(A_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < BR; i++)
		B.push_back(vector<float> (B_def[i], B_def[i]+sizeof(B_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < CR; i++)
		C.push_back(vector<float> (C_def[i], C_def[i]+sizeof(C_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < DR; i++)
		D.push_back(vector<float> (D_def[i], D_def[i]+sizeof(D_def[i])/sizeof(float)));
	
	for (unsigned int i = 0; i < XR; i++)
		X.push_back(vector<float> (X_def[i], X_def[i]+sizeof(X_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < YR; i++)
		Y.push_back(vector<float> (Y_def[i], Y_def[i]+sizeof(Y_def[i])/sizeof(float)));

	U=U_def;
	
	E=matrix_sub(U, Y);
}

SSController::SSController(vector<vector<float> > A_def, vector<vector<float> > B_def, vector<vector<float> > C_def, vector<vector<float> > D_def, vector<vector<float> > U_def)
{
	A=A_def;
	B=B_def;
	C=C_def;
	D=D_def;
	U=U_def;
	
	float X_def[XR][XC] = {{0}};

	float Y_def[YR][YC] = {{0}};
	
	for (unsigned int i = 0; i < XR; i++)
		X.push_back(vector<float> (X_def[i], X_def[i]+sizeof(X_def[i])/sizeof(float)));

	for (unsigned int i = 0; i < YR; i++)
		Y.push_back(vector<float> (Y_def[i], Y_def[i]+sizeof(Y_def[i])/sizeof(float)));
	
	E=matrix_sub(U, Y);
	
}

void SSController::setU(vector<vector<float> > U_def)
{
	U = U_def;
}

void SSController::setU(float U_def)
{
	U[0][0] = U_def;
}

vector<vector<float> > SSController::getU()
{
	return U;
}

vector<vector<float> > SSController::getE()
{
	return E;
}

vector<vector<float> > SSController::update()
{
	vector<vector<float> > AX = matrix_multiply(A, X);
	vector<vector<float> > BU = matrix_multiply(B, E);
	vector<vector<float> > CX = matrix_multiply(C, X);
	vector<vector<float> > DU = matrix_multiply(D, E);
	Y = matrix_add(CX, DU);
	X = matrix_add(AX, BU);	
	E = matrix_sub(U, Y);
	
	return Y;
}

vector<vector<float> > SSController::getX()
{
	return X;
}

vector<vector<float> > SSController::getY()
{
	return Y;
}

