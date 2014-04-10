#include <iostream> 
#include <stdio.h>
#include <vector>

using namespace std;

void matrix_print(vector<vector<float> > M)
{
	for(unsigned int i = 0; i < M.size(); i++)
	{
		for(unsigned int j = 0; j < M[i].size(); j++)
		{
			cout << M[i][j] << " ";
		}
		cout << endl;
	}
}
	
vector<vector<float> > matrix_multiply(vector<vector<float> > M1, vector<vector<float> > M2)
{

	if(M1.at(0).size()!=M2.size())
		cout << "Invalid operation, inner matrix dimensions must match";
	vector<vector<float > > A;
	A.resize(M1.size(), vector<float>(M2[0].size(), 0));
	for (unsigned int i = 0; i < A.size(); i++) //rows
	{
		for(unsigned int j = 0; j < A.at(0).size(); j++) //columns
		{
			for(unsigned int k = 0; k < M1.at(0).size(); k++)
			{
				A[i][j] += M1[i][k]*M2[k][j];
			}
		}
	}

	return A;
}

vector<vector<float> > matrix_add(vector<vector<float> > M1, vector<vector<float> > M2)
{
	vector<vector<float> > A;

	if((M1.size()==M2.size())&&(M2[0].size()==M1[0].size()))
	{
		A.resize(M2.size(), vector<float>(M2[0].size(), 0));
		for(unsigned int i = 0; i < M2.size(); i++)
		{
			for(unsigned int j = 0; j < M2[0].size(); j++)
			{
				A[i][j] = M1[i][j] + M2[i][j];
			}
		}
	}
	return A;
}

vector<vector<float> > matrix_sub(vector<vector<float> > M1, vector<vector<float> > M2)
{
	vector<vector<float> > A;

	if((M1.size()==M2.size())&&(M2[0].size()==M1[0].size()))
	{
		A.resize(M2.size(), vector<float>(M2[0].size(), 0));
		for(unsigned int i = 0; i < M2.size(); i++)
		{
			for(unsigned int j = 0; j < M2[0].size(); j++)
			{
				A[i][j] = M1[i][j] - M2[i][j];
			}
		}
	}
	return A;
}


