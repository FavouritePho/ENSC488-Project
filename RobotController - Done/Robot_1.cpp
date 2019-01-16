//  Feb 18, 2017 - Steven: Changed made to Robot, InverseKin Function, ForwardKin - Steven
//  Feb 21, 2017 - Steven: Revised some D-H Parameters

#include "StdAfx.h"
#include "Robot.h"
#include "matrix.h"
#include <iostream>
#include <conio.h>
#include "InverseKinResult.h"
#include "ensc-488.h"
#include <math.h>
using namespace std;

double currentPosition[4] = { 90, 0, 0, 0 }; // Intial configuration of robot 
double final_position[4];
double minimized_rotation[4];
Arm Arms[4];

double min1 = -150;
double max1 = 150;
double min2 = -100;
double max2 = 100;
double min3 = -200;
double max3 = -100;
double min4 = -160;
double max4 = 160;

Robot::Robot(void)
{
	Arm arm1, arm2, arm3, arm4;    // changed D-H parameters
	arm1.SetValues(0, 0, 405, 0);
	arm2.SetValues(0, 195, 70, 0); 
	arm3.SetValues(0, 142, 0, 0);
	arm4.SetValues(0, 0, -140, 0);
	
	Arms[0] = arm1;
	Arms[1] = arm2;
	Arms[2] = arm3;
	Arms[3] = arm4;
}

InverseKinResult* Robot::InverseKin(double x, double y, double z, double phi)
{
	InverseKinResult * result = new InverseKinResult();
	double ** allResults = new double*[100];

	// todo - put some useful information in message
	string message = "Found a solution";

	// put the algorithm for inverse kin here

	// Declare constants
	double px = x;
	double py = y;
	double pz = z;
	double theta1 = 0;
	double theta2 = 0;
	double d3 = 0;
	double theta4 = 0; 
	double _PHI = phi;
	double a1 = 195;
	double a2 = 142;
	
	// C2 = (px^2+py^2-a1^2-a2^2)/(2a1*a2)
	// S2 = sqrt(1-C2^2)
	
	double C_2 = (px*px + py*px - a1*a1 - a2*a2)/(2*(a1*a2));
	double S_2 = sqrt(1 - C_2*C_2);

	// Calculate Theta values and d3 Value
	theta2 = atan2(S_2, C_2) *(180/PI);
	theta1 = atan2(py, px) - atan2(a1*S_2, a1 - a2*C_2) *(180 / PI);
	theta4 = theta1 + theta2 - _PHI;
	d3 = 335 - pz ; // D3 = D1+D2-D4-D5-PZ 

	// currently set up to return a list of solutions from this function
	// if list of solutions is returned need to add another function to calculate closest solution
	// could change this function to return only one solution (which is the closest)
	// could also just leave it so that it returns a list and only return list with one solution only (the closest one)

	// to test this function:
	// run program
	// enter 'M' for move
	// enter 'X'
	// enter xyz coordinates (enter one, press enter, enter the next ... etc)
	// console will display the joint parameters that are calculated and give option to move there
	// for debugging purposes might be useful to print things to console
	
	/*if (abs(theta2) >= 0)
	{
		number_of_solutions = 1;
	}
	//Checking for other solutions
	if (abs(theta2) + 90 <= 150)
	{
		number_of_solutions++;
	}*/

	
	// Calculate Shortest Route
	// Find the difference in rotation and pick the smallest value
	// Solution number i corresponds to which solution set is the smallest value

	if (!GetConfiguration(currentPosition))
	{
		cout << "Error communicating with robot. Aborting move. \n";
	}

	for (int i = 0; i < 4; i++)
	{
		minimized_rotation[i] = final_position[i] - currentPosition[i];
	}

	double smallest = minimized_rotation[0];
	double solution_number;
	for (int i = 0; i < 3; i++)
	{
		if (minimized_rotation[i] < smallest)
		{
			smallest = minimized_rotation[i];
			solution_number = i;
		}
	}
	

	// look in Arm.cpp for example of how to use matrix library

	cout << "\nTheta1 = " << theta1;
	cout << "\nTheta2 = " << theta2;
	cout << "\nd3 = " << d3;
	cout << "\nTheta4 = " << theta4 << "\n";
	
	// example of how to add result to list
	//--------------------------------------
	double * result1 = new double[4];
	result1[0] = theta1;
	result1[1] = theta2;
	result1[2] = (d3-410);
	result1[3] = theta4;

	allResults[0] = result1;
	//--------------------------------------
	// -------------------------------------
	
	result -> SetResults(allResults);
	result -> SetMessage(message);

	// Update Current position
	UpdatePosition(theta1, theta2, d3, theta4);

	return result;
}

double * Robot::ClosestSolution(double ** allSolutions, int solutionCount)
{
	int inRangeSolutionsCount = 0;
	double solutionsToCheckCount;
	double **inRangeSolutions;
	double **solutionsToCheck;

	for (int j = 0; j < solutionCount; j++)
	{
		if (allSolutions[j][0] < min1 || allSolutions[j][0] > max1)
		{
			break;
		}
		if (allSolutions[j][1] < min2 || allSolutions[j][1] > max2)
		{
			break;
		}
		if (allSolutions[j][2] < min3 || allSolutions[j][2] > max3)
		{
			break;
		}
		if (allSolutions[j][3] < min4 || allSolutions[j][3] > max4)
		{
			break;
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

	double * solution = new double[4];

	solution[0] = allSolutions[shortestSolution][0];
	solution[1] = allSolutions[shortestSolution][1];
	solution[2] = allSolutions[shortestSolution][2];
	solution[3] = allSolutions[shortestSolution][3];

	if (inRangeSolutionsCount == 0)
	{
		CoerceSolutionInRange(solution);
	}

	return solution;
}

void Robot::CoerceSolutionInRange(double * solution)
{
	if (solution[0] < min1 || solution[0] > max1)
	{
		if (abs(solution[0] - min1) < abs(solution[0] - max1))
		{
			solution[0] = min1;
		}
		else
		{
			solution[0] = max1;
		}
	}
	
	if (solution[1] < min2 || solution[1] > max2)
	{
		if (abs(solution[1] - min2) < abs(solution[1] - max2))
		{
			solution[1] = min2;
		}
		else
		{
			solution[1] = max2;
		}
	}
	
	if (solution[2] < min3 || solution[2] > max3)
	{
		if (abs(solution[2] - min3) < abs(solution[2] - max3))
		{
			solution[2] = min3;
		}
		else
		{
			solution[2] = max3;
		}
	}
	
	if (solution[3] < min4 || solution[3] > max4)
	{
		if (abs(solution[3] - min4) < abs(solution[3] - max4))
		{
			solution[3] = min4;
		}
		else
		{
			solution[3] = max4;
		}
	}
}

double* Robot::ForwardKin(double j1, double j2, double j3, double j4)
{
	Arms[0].SetValues(0, 0, 405, j1);          //changed D-H Parameter
	Arms[1].SetValues(0, 195, 70, j2);
	Arms[2].SetValues(0, 142, -(410+j3), 0);
	Arms[3].SetValues(0, 0, -140, j4);

	Arms[0].GenerateMatrix();
	Arms[1].GenerateMatrix();
	Arms[2].GenerateMatrix();
	Arms[3].GenerateMatrix();

	cout << "\nLink 1 \n";
	PrintMatrix(*Arms[0].myMatrix);
	
	cout << "\nLink 2 \n";
	PrintMatrix(*Arms[1].myMatrix);
	
	cout << "\nLink 3 \n";
	PrintMatrix(*Arms[2].myMatrix);
	
	cout << "\nLink 4 \n";
	PrintMatrix(*Arms[3].myMatrix);
		

	Matrix<double> transformMatrix = (*Arms[0].myMatrix) * (*Arms[1].myMatrix) * (*Arms[2].myMatrix)* (*Arms[3].myMatrix);

	double *position;
	position = new double[3];
	position[0] = transformMatrix.get(0,3);
	position[1] = transformMatrix.get(1,3);
	position[2] = transformMatrix.get(2,3);

	UpdatePosition(j1, j2, j3, j4);

	return position;
}

void Robot::UpdatePosition(double t1, double t2, double t3, double t4)
{
	currentPosition[0] = t1;
	currentPosition[1] = t2;
	currentPosition[2] = t4;
};

void Robot::MinimizedRotation(double array_1[3], double array_2[3])
{

};

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
