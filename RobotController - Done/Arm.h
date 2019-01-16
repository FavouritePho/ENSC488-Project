#pragma once

#include"matrix.h"
class Arm
{
public:
	Arm(void);

	double AlphaMinusOne;
	double AMinusOne;
	double D;
	double Theta;
	void SetValues(double alphaMinusOne, double aMinusOne, double d, double theta);
	Matrix<double> * myMatrix;
	void GenerateMatrix();

	~Arm(void);
};

