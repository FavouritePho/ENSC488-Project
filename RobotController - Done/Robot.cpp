//  Feb 18, 2017 - Steven: Changed made to Robot, InverseKin Function, ForwardKin - Steven
//  Feb 21, 2017 - Steven: Revised some D-H Parameters

#include "StdAfx.h"
#include "Robot.h"
#include "matrix.h"
#include <iostream>
#include <conio.h>
#include <string>
double degreeToRad = (PI / 180);
double radToDegree = (180 / PI);

using namespace std;

Arm Arms[4];

	double Min1 = -150;
	double Max1 = 150;
	double Min2 = -100;
	double Max2 = 100;
	double Min3 = -200;
	double Max3 = -100;
	double Min4 = -160;
	double Max4 = 160;



Robot::Robot(void)
{
	Arm arm1, arm2, arm3, arm4;   
	arm1.SetValues(0, 0, 405, 0);
	arm2.SetValues(0, 195, 70, 0); 
	arm3.SetValues(0, 142, 0, 0);
	arm4.SetValues(0, 0, -140, 0);
	
	Arms[0] = arm1;
	Arms[1] = arm2;
	Arms[2] = arm3;
	Arms[3] = arm4;
}

double * Robot::ClosestSolution(double ** allSolutions, double * currentPosition, int solutionCount)
{
	if (solutionCount == 1)
	{
		cout << "\n" << "One solution found \n";
		cout << "Solution 1: theta1 = " << allSolutions[0][0] << " theta2 = " << allSolutions[0][1];
		cout << " d3 = " << allSolutions[0][2] << " theta 4 = " << allSolutions[0][3] << "\n";
	}
	else
	{
		cout << "\n" << "Two solutions found \n";
		cout << "Solution 1: theta1 = " << allSolutions[0][0] << " theta2 = " << allSolutions[0][1];
		cout << " d3 = " << allSolutions[0][2] << " theta 4 = " << allSolutions[0][3] << "\n";
		cout << "Solution 2: theta1 = " << allSolutions[1][0] << " theta2 = " << allSolutions[1][1];
		cout << " d3 = " << allSolutions[1][2] << " theta 4 = " << allSolutions[1][3] << "\n";
	}

	int inRangeSolutionsCount = 0;
	int solutionsToCheckCount;
	double ** inRangeSolutions = new double*[2];
	double ** solutionsToCheck;

	cout << "Current Position: theta 1 = " << currentPosition[0] << " theta2 = " << currentPosition[1];
	cout << " d3 = " << currentPosition[2] << " theta4 = " << currentPosition[3] << " \n";

	double * solution = new double[4];

	for (int j = 0; j < solutionCount; j++)
	{
		double var = allSolutions[j][0];
		if (allSolutions[j][0] < Min1 || allSolutions[j][0] > Max1)
		{
			continue;
		}
		if (allSolutions[j][1] < Min2 || allSolutions[j][1] > Max2)
		{
			continue;
		}
		if (allSolutions[j][2] < Min3 || allSolutions[j][2] > Max3)
		{
			continue;
		}
		if (allSolutions[j][3] < Min4 || allSolutions[j][3] > Max4)
		{
			continue;
		}

		inRangeSolutions[inRangeSolutionsCount] = allSolutions[j];

		inRangeSolutionsCount++;
	}

	if (inRangeSolutionsCount == 0)
	{
		solutionsToCheck = allSolutions;
		solutionsToCheckCount = solutionCount;
	}
	else
	{
		solutionsToCheck = inRangeSolutions;
		solutionsToCheckCount = inRangeSolutionsCount;
	}

	double shortestPath = 0;
	double currentPath = 0;
	int shortestSolution = 0;

	for(int i = 0; i < 4; i++)
	{
		shortestPath += (solutionsToCheck[0][i] - currentPosition[i]) * (solutionsToCheck[0][i] - currentPosition[i]);
	}

	for(int i = 1; i < solutionsToCheckCount; i++)
	{
		currentPath = 0;
		
		for(int j = 0; j < 4; j++)
		{
			currentPath += (solutionsToCheck[i][j] - currentPosition[j]) * (solutionsToCheck[i][j] - currentPosition[j]);
		}

		if (currentPath < shortestPath)
		{
			shortestPath = currentPath;
			shortestSolution = i;
		}
	}

	solution[0] = allSolutions[shortestSolution][0];
	solution[1] = allSolutions[shortestSolution][1];
	solution[2] = allSolutions[shortestSolution][2];
	solution[3] = allSolutions[shortestSolution][3];

	if (solutionCount == 2)
	{
		if (shortestSolution == 0)
		{
			cout << "Solution 1 is closest to current position \n";
		}
		else
		{
			cout << "Solution 2 is closest to current position \n";
		}
	}

	if (inRangeSolutionsCount == 0)
	{
		CoerceSolutionInRange(solution);
		cout << "******WARNING******* \n SOLUTION OUT OF RANGE \n ";
		cout << "\n" << "Solution is outside of joint limits, moving insead to: \n";
		cout << "theta1 = " << solution[0] << ", theta2 = " << solution[1];
		cout << ", d3 = " << solution[2] << ", theta4 = " << solution[3] << "\n";
	}

	return solution;
}

void Robot::CoerceSolutionInRange(double * solution)
{
	if (solution[0] < Min1 || solution[0] > Max1)
	{
		if (abs(solution[0] - Min1) < abs(solution[0] - Max1))
		{
			solution[0] = Min1;
		}
		else
		{
			solution[0] = Max1;
		}
	}
	
	if (solution[1] < Min2 || solution[1] > Max2)
	{
		if (abs(solution[1] - Min2) < abs(solution[1] - Max2))
		{
			solution[1] = Min2;
		}
		else
		{
			solution[1] = Max2;
		}
	}
	
	if (solution[2] < Min3 || solution[2] > Max3)
	{
		if (abs(solution[2] - Min3) < abs(solution[2] - Max3))
		{
			solution[2] = Min3;
		}
		else
		{
			solution[2] = Max3;
		}
	}
	
	if (solution[3] < Min4 || solution[3] > Max4)
	{
		if (abs(solution[3] - Min4) < abs(solution[3] - Max4))
		{
			solution[3] = Min4;
		}
		else
		{
			solution[3] = Max4;
		}
	}
}



InverseKinResult* Robot::InverseKin(double x, double y, double z, double phi)
{
	InverseKinResult * result = new InverseKinResult();
	double ** allResults = new double*[100];
	string message = "";
	int solutionCount = 0;

	// Check if frame is out of max range
	if (sqrt(x*x + y*y) > 337)
	{
		cout << "Selected position x = " << x << ", y = " << y << " is out of workspace, violates external joint limits. \n";
		
		double theta = atan(x/y);
		x = 337 * sin(theta);
		y = 337 * cos(theta);
		cout << "Moving to x = " << x << ", y = " << y << "\n";
		solutionCount = 1;
	}

	if (sqrt(x*x + y*y) > 336.8)
	{
		solutionCount = 1;
	}

	//Check if frame is out of min range
	if (sqrt(x*x + y*y) < 110)
	{
		cout << "Selected position x = " << x << ", y = " << y << " is out of workspace, violates internal joint limits. \n";
		double theta = atan(x / y);
		x = 110 * sin(theta);
		y = 110 * cos(theta);
		cout << "Moving to x = " << x << ", y = " << y << "\n";
	}

	double px = x;
	double py = y;
	double pz = z;
	double _PHI = phi;
	double a1 = 195;
	double a2 = 142;
	
	double * solution1 = new double[4];
	double * solution2 = new double[4];
		
	double c2 = ((px*px) + (py*py) - (a1*a1) - (a2*a2))/(2*a1*a2);
	double s2 = -abs(sqrt(1 - c2*c2));								// todo catch case 1-c2*c2 is negative

	double s1Solution1 = ((a2*s2)*px + (a1 + a2*c2)*py) / (pow((a2*s2), 2) + pow((a1 + a2*c2), 2));
	double c1Solution1 = ((a1 + a2*c2)*px - (a2*s2)*py) / (pow((a2*s2), 2) + pow((a1 + a2*c2), 2));

	double s1Solution2 = ((a2*(-s2))*px + (a1 + a2*c2)*py) / (pow((a2*(-s2)), 2) + pow((a1 + a2*c2), 2));
	double c1Solution2 = ((a1 + a2*c2)*px - (a2*(-s2))*py) / (pow((a2*(-s2)), 2) + pow((a1 + a2*c2), 2));

	double d3 = (335-pz)-410;

	solution1[0] = atan2(s1Solution1, c1Solution1) * radToDegree;					//theta1
	solution1[1] = -atan2(s2, c2) * radToDegree;									//theta2
	solution1[2] = d3; 
	solution1[3] = (solution1[0] + solution1[1]) - _PHI;							//theta4

	solution2[0] = atan2(s1Solution2, c1Solution2) * radToDegree;					//theta1
	solution2[1] = -atan2(-s2, c2) * radToDegree;									//theta2
	solution2[2] = d3; 
	solution2[3] = (solution2[0] + solution2[1]) - _PHI;							//theta4

	allResults[0] = solution1;
	allResults[1] = solution2;

	if (solutionCount == 1)
	{
		result -> SetResults(allResults, 1);
	}
	else
	{
		result -> SetResults(allResults, 2);
	}
	result -> SetMessage(message);

	return result;
}

double* Robot::ForwardKin(double j1, double j2, double j3, double j4)
{

	Arms[0].SetValues(0, 0, 405, j1);         
	Arms[1].SetValues(0, 195, 70, j2);
	Arms[2].SetValues(0, 142, -(410+j3), 0);
	Arms[3].SetValues(0, 0, -140, j4);

	Arms[0].GenerateMatrix();
	Arms[1].GenerateMatrix();
	Arms[2].GenerateMatrix();
	Arms[3].GenerateMatrix();		

	Matrix<double> transformMatrix = (*Arms[0].myMatrix) * (*Arms[1].myMatrix) * (*Arms[2].myMatrix)* (*Arms[3].myMatrix);

	double _PHI = j1 + j2 - j4;						
	double *position;
	position = new double[4];
	position[0] = transformMatrix.get(0,3);
	position[1] = transformMatrix.get(1,3);
	position[2] = transformMatrix.get(2,3);
	position[3] = _PHI;

	return position;
}

void Robot::PrintMatrix(Matrix<double> matrix)
{
	cout << matrix.get(0,0) << "  " << matrix.get(0,1) << "  " << matrix.get(0,2) << "  " << matrix.get(0,3) << "\n";
	cout << matrix.get(1,0) << "  " << matrix.get(1,1) << "  " << matrix.get(1,2) << "  " << matrix.get(1,3) << "\n";
	cout << matrix.get(2,0) << "  " << matrix.get(2,1) << "  " << matrix.get(2,2) << "  " << matrix.get(2,3) << "\n";
	cout << matrix.get(3,0) << "  " << matrix.get(3,1) << "  " << matrix.get(3,2) << "  " << matrix.get(3,3) << "\n";
}


Robot::~Robot(void)
{
}
