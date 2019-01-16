#include "StdAfx.h"
#include "Arm.h"
#include<math.h>
#include "stdafx.h"
#include <conio.h>

Arm::Arm(void)
{
}

void Arm::SetValues(double alphaMinusOne, double aMinusOne, double d, double theta)
{
	AlphaMinusOne = alphaMinusOne;
	AMinusOne = aMinusOne;
	D = d;
	Theta = theta;
}

void Arm::GenerateMatrix()
{
	myMatrix = new Matrix<double>(4,4);
	myMatrix -> put(0, 0, cos(Theta));
	myMatrix -> put(0, 1, -sin(Theta));
	myMatrix -> put(0, 2, 0);
	myMatrix -> put(0, 3, AMinusOne);

	myMatrix -> put(1, 0, sin(Theta)*cos(AlphaMinusOne));
	myMatrix -> put(1, 1, cos(Theta)*cos(AlphaMinusOne));
	myMatrix -> put(1, 2, -sin(AlphaMinusOne));
	myMatrix -> put(1, 3, -sin(AlphaMinusOne) * D);

	myMatrix -> put(2, 0, sin(Theta)*sin(AlphaMinusOne));
	myMatrix -> put(2, 1, cos(Theta)*sin(AlphaMinusOne));
	myMatrix -> put(2, 2, cos(AlphaMinusOne));
	myMatrix -> put(2, 3, cos(AlphaMinusOne)*D);

	myMatrix -> put(3, 0, 0);
	myMatrix -> put(3, 1, 0);
	myMatrix -> put(3, 2, 0);
	myMatrix -> put(3, 3, 1);
}


Arm::~Arm(void)
{
}
