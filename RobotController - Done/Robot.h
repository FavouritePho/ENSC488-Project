// Feb 18,2017 - Steven: Changed made to InverseKin

#pragma once

#include"Arm.h"
#include"matrix.h"
#include "InverseKinResult.h"
#include "ensc-488.h"
#include "RobotPath.h"

class Robot
{
public:
	Robot(void);
	double* ForwardKin(double j1, double j2, double j3, double j4);
	void PrintMatrix(Matrix<double> matrix);
	InverseKinResult* InverseKin(double x, double y, double z, double phi); // Added Phi to arguments
	double* ClosestSolution(double ** allSolutions, double * currentPosition, int solutionCount);
	void CoerceSolutionInRange(double * solution);
	~Robot(void);
};

