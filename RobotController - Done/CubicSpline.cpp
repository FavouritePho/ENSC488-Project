#include "StdAfx.h"
#include "CubicSpline.h"
#include "matrix.h"

#include <string>
#include <conio.h>
#include <math.h>
#include <iostream>

using namespace std;

double * velocitiesAtPoints;
double * segmentTimes;
const int numSegments = 4;


CubicSpline::CubicSpline(double * points, double * times, int count)
{
	segmentTimes = times;
	velocitiesAtPoints = new double[numSegments];
	allCoeffs = new double*[numSegments];

	velocitiesAtPoints[0] = 0;
	velocitiesAtPoints[4] = 0;

	for (int i = 1; i < numSegments; i++)
	{
		velocitiesAtPoints[i] = ((points[i] - points[i-1])/segmentTimes[i-1] + (points[i+1] - points[i])/segmentTimes[i]) / 2;
	}

	for(int i = 0; i< numSegments; i++)
	{
		double * coeffs = CalculateCoefficients(points, i);
		allCoeffs[i] = coeffs;
		double a = coeffs[0];
		double b = coeffs[1];
		double c = coeffs[2];
		double d = coeffs[3];
	}
}

double * CubicSpline::CalculateCoefficients(double * points, int segment)
{
	double * coeffs = new double[4];
	
	double t0 = 0;

	for (int i = 0; i < segment;  i++)
	{
		t0 += segmentTimes[i];
	}

	double t1 = t0 + segmentTimes[segment];

	double v0 = velocitiesAtPoints[segment];
	double v1 = velocitiesAtPoints[segment + 1];

	double p0 = points[segment];
	double p1 = points[segment+1];

	Matrix<double> * V = new Matrix<double>(4,4);
	Matrix<double> Vi;
	Matrix<double> P(4,1);
	Matrix<double> A;

	V -> put(0, 0, t0*t0*t0);
	V -> put(0, 1, t0*t0);
	V -> put(0, 2, t0);
	V -> put(0,3, 1);

	V -> put(1, 0, t1*t1*t1);
	V -> put(1, 1, t1*t1);
	V -> put(1, 2, t1);
	V -> put(1, 3, 1);

	V -> put(2, 0, 3*t0*t0);
	V -> put(2, 1, 2*t0);
	V -> put(2, 2, 1);
	V -> put(2, 3, 0);

	V -> put(3, 0, 3*t1*t1);
	V -> put(3, 1, 2*t1);
	V -> put(3, 2, 1);
	V -> put(3, 3, 0);

	P.put(0, 0, p0);
	P.put(1, 0, p1);
	P.put(2, 0, v0);
	P.put(3, 0, v1);

	Vi = V -> getInverse();
	A = Vi * P;

	coeffs[0] = A.get(0,0);
	coeffs[1] = A.get(1,0);
	coeffs[2] = A.get(2,0);
	coeffs[3] = A.get(3,0);

	return coeffs;
}

double CubicSpline::GetYAtX(double x)
{
	double y = 0;
	double cumulativeTime = 0;

	for (int i = 0; i <= numSegments; i++)
	{
		cumulativeTime += segmentTimes[i];
	
		if ((x < cumulativeTime) && (x >= cumulativeTime - segmentTimes[i]))
		{
			y = x*x*x*allCoeffs[i][0] + x*x*allCoeffs[i][1] + x*allCoeffs[i][2] + allCoeffs[i][3];
		}
	}

	return y;
}

double CubicSpline::GetVelocityAtX(double x)
{
	double y = 0;
	double cumulativeTime = 0;

	for (int i = 0; i <= numSegments; i++)
	{
		cumulativeTime += segmentTimes[i];
	
		if ((x < cumulativeTime) && (x >= cumulativeTime - segmentTimes[i]))
		{
			y = 3*x*x*allCoeffs[i][0] + 2*x*allCoeffs[i][1] + allCoeffs[i][2]; 
		}
	}

	return y;
}

double CubicSpline::GetAccelerationAtX(double x)
{
	double y = 0;
	double cumulativeTime = 0;

	for (int i = 0; i <= numSegments; i++)
	{
		cumulativeTime += segmentTimes[i];

		if ((x < cumulativeTime) && (x >= cumulativeTime - segmentTimes[i]))
		{
			y = 6*x*allCoeffs[i][0] + 2*allCoeffs[i][1];
		}
	}

	return y;
}

CubicSpline::~CubicSpline(void)
{
}
