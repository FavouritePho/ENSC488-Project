#pragma once
class CubicSpline
{
public:
	CubicSpline(double * points, double * times, int count);
	double GetVelocityAtX(double x);
	double GetAccelerationAtX(double x);
	double * CalculateCoefficients(double * points, int segment);
	double ** allCoeffs;
	double GetYAtX(double x);
	~CubicSpline(void);
};

