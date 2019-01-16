#include "StdAfx.h"
#include "RobotPath.h"
#include <time.h>

Robot * robotP;

using namespace std;

RobotPath::RobotPath(double ** cartesianConfigs, double totalTime, int numConfigs)
{
	SetVelocityAndAccelerationLimits();
	Robot * robotP = new Robot();
	CartesianConfigs = cartesianConfigs;
	TotalTime = totalTime;
	PositionCount = numConfigs;
	CalculateJointConfigs();
	Times = CalculateTimes();;
	CalculateSplines();
}

void RobotPath::SetVelocityAndAccelerationLimits()
{
	double * velocityLimit1 = new double[2];
	double * velocityLimit2 = new double[2];
	velocityLimit1[0] = -150;
	velocityLimit1[1] = 150;
	velocityLimit2[0] = -50;
	velocityLimit2[1] = 50;

	double * accelLimit1 = new double[2];
	double * accelLimit2 = new double[2];

	accelLimit1[0] = -600;
	accelLimit1[1] = 600;

	accelLimit2[0] = -200;
	accelLimit2[1] = 200;

	velocityLimits = new double*[4];
	accelLimits = new double*[4];

	velocityLimits[0] = velocityLimit1;
	velocityLimits[1] = velocityLimit1;
	velocityLimits[2] = velocityLimit2;
	velocityLimits[3] = velocityLimit1;

	accelLimits[0] = accelLimit1;
	accelLimits[1] = accelLimit1;
	accelLimits[2] = accelLimit2;
	accelLimits[3] = accelLimit1;

	return;
}


double * RobotPath::CalculateTimes()
{
	double * longestJointMove = new double[4];
	double * times = new double[4];

	for (int i = 0; i < PositionCount - 1; i++)
	{
		double longestMove = 0;
		for (int j = 0; j < 4; j++)
		{
			if (abs(JointConfigs[i][j] - JointConfigs[i+1][j]) > longestMove)
			{
				longestMove = abs(JointConfigs[i][j] - JointConfigs[i+1][j]);
			}
		}

		longestJointMove[i] = longestMove;
	}

	double totalLongMoves = 0;
	
	for (int i = 0; i < PositionCount - 1; i++)
	{
		totalLongMoves += longestJointMove[i];
	}

	for (int i = 0; i < PositionCount - 1; i++)
	{
		times[i] = TotalTime * longestJointMove[i] / totalLongMoves;
	}

	return times;
}

void RobotPath::CalculateJointConfigs()
{
	double currentPosition[4];
	double * currentP;
	JointConfigs = new double*[5];
	Theta1s = new double[5];
	Theta2s = new double[5];
	Theta4s = new double[5];
	D3s = new double[5];

	GetConfiguration(currentPosition);
    InverseKinResult * p1Result = robotP -> InverseKin(CartesianConfigs[0][0], CartesianConfigs[0][1], CartesianConfigs[0][2], CartesianConfigs[0][3]);
	double * p1 = robotP -> ClosestSolution(p1Result -> Configurations, currentPosition, p1Result -> SolutionCount);

	currentP = new double[4];

	for (int i = 0; i < 4; i++)
	{
		currentP[i] = currentPosition[i];
	}

	JointConfigs[0] = currentP;
	JointConfigs[1] = p1;

	for (int i = 2; i < PositionCount; i++)
	{
		InverseKinResult * result = robotP -> InverseKin(CartesianConfigs[i-1][0], CartesianConfigs[i-1][1], CartesianConfigs[i-1][2], CartesianConfigs[i-1][3]);
		double * p = robotP -> ClosestSolution(result-> Configurations, JointConfigs[i-1], result -> SolutionCount);
		JointConfigs[i] = p;
	}

	for (int i = 0; i < PositionCount; i++)
	{
		Theta1s[i] = JointConfigs[i][0];
		Theta2s[i] = JointConfigs[i][1];
		Theta4s[i] = JointConfigs[i][2];
		D3s[i] = JointConfigs[i][3];
	}
}

void RobotPath::CalculateSplines()
{
	Theta1Spline = new CubicSpline(Theta1s, Times, 4);
	Theta2Spline = new CubicSpline(Theta2s, Times, 4);
	D3Spline = new CubicSpline(D3s, Times, 4);
	Theta4Spline = new CubicSpline(Theta4s, Times, 4);
}

void RobotPath::ExecutePath(string fileName)
{
	double outputFileTime = 0;
	double time = 0;
	double timeIncrement = 0.02;
	double waitTime = timeIncrement;
	int configurationCount = static_cast<int>(ceil(TotalTime/timeIncrement));
	double currentPosition[4];
	GetConfiguration(currentPosition);
	double deltaVelocity = 0;

	double currentVelocities[4] = {0,0,0,0};
	double nextVelocities[4];
	double nextAccelerations[4];
	double nextPosition[4];

	nextPosition[0] = Theta1Spline -> GetYAtX(time);
	nextPosition[1] = Theta2Spline -> GetYAtX(time);
	nextPosition[2] = Theta4Spline -> GetYAtX(time);
	nextPosition[3] = D3Spline -> GetYAtX(time);

	ofstream executedPositionJointSpaceFile;
	ofstream positionErrorJointSpaceFile;
	ofstream plannedPositionJointSpaceFile;
	executedPositionJointSpaceFile.open(fileName + "ExecutedPositionJointSpace.csv");
	positionErrorJointSpaceFile.open(fileName + "PositionErrorJointSpace.csv");
	plannedPositionJointSpaceFile.open(fileName + "PlannedPositionJointSpace.csv");
		
	for (int i = 0; i < 4; i++)
	{
		nextVelocities[i] = (nextPosition[i] - currentPosition[i]) / timeIncrement;
		
		if (nextVelocities[i] < velocityLimits[i][0])
		{
			cout << "Velocity limit of joint " << i << " exceeded. Velocity set to max value " << velocityLimits[i][0] << "\n";
			nextVelocities[i] = velocityLimits[i][0];
			if (abs((nextPosition[i] - currentPosition[i])/nextVelocities[i]) > waitTime)
			{
				waitTime = abs((nextPosition[i] - currentPosition[i])/nextVelocities[i]);
			}
		}
		else if (nextVelocities[i] > velocityLimits[i][1])
		{
			cout << "Velocity limit of joint " << i << " exceeded. Velocity set to max value " << velocityLimits[i][1] << "\n";
			nextVelocities[i] = velocityLimits[i][1];
			if (abs((nextPosition[i] - currentPosition[i])/nextVelocities[i]) > waitTime)
			{
				waitTime = abs((nextPosition[i] - currentPosition[i])/nextVelocities[i]);
			}
		}
	}

	for (int i = 0; i < 4; i++)
	{
		nextAccelerations[i] = (nextVelocities[i] - currentVelocities[i])/timeIncrement;
	}

	for (int i = 0; i < 4; i++)
	{
		currentVelocities[i] = nextVelocities[i];
	}

	MoveWithConfVelAcc(nextPosition, nextVelocities, nextAccelerations);

	Sleep(waitTime * 1000);
	waitTime = timeIncrement;

	for (int i = 1; i < configurationCount; i++)
	{
		time += timeIncrement;

		GetConfiguration(currentPosition);

		nextPosition[0] = Theta1Spline -> GetYAtX(time);
		nextPosition[1] = Theta2Spline -> GetYAtX(time);
		nextPosition[2] = Theta4Spline -> GetYAtX(time);
		nextPosition[3] = D3Spline -> GetYAtX(time);

		executedPositionJointSpaceFile << outputFileTime << ", " << currentPosition[0]  << ", " << currentPosition[1] << ", " << currentPosition[2] << ", " << currentPosition[3] << "\n";
		plannedPositionJointSpaceFile << outputFileTime << ", " << nextPosition[0]  << ", " << nextPosition[1] << ", " << nextPosition[2] << ", " << nextPosition[3] << "\n";
		positionErrorJointSpaceFile << outputFileTime << ", " << (currentPosition[1] - nextPosition[1])  << ", " << (currentPosition[2] - nextPosition[2])  << ", " << (currentPosition[2] - nextPosition[2])  << ", " << (currentPosition[3] - nextPosition[3]) << "\n";

		for (int j = 0; j < 4; j++)
		{
			nextVelocities[j] = (nextPosition[j] - currentPosition[j]) / timeIncrement;
			if (nextVelocities[j] < velocityLimits[j][0])
			{
				cout << "Velocity limit of joint " << j << " exceeded. Velocity set to max value " << velocityLimits[j][0] << "\n";
				nextVelocities[j] = velocityLimits[j][0];
			
				if (abs((nextPosition[j] - currentPosition[j])/nextVelocities[j]) > waitTime)
				{
					waitTime = abs((nextPosition[j] - currentPosition[j])/nextVelocities[j]);
				}
			}
			else if (nextVelocities[j] > velocityLimits[j][1])
			{
				cout << "Velocity limit of joint " << j << " exceeded. Velocity set to max value " << velocityLimits[j][1] << "\n";
				nextVelocities[j] = velocityLimits[j][1];
				if (abs((nextPosition[j] - currentPosition[j])/nextVelocities[j]) > waitTime)
				{
					waitTime = abs((nextPosition[j] - currentPosition[j])/nextVelocities[j]);
				}
			}
		}

		for (int j = 0; j < 4; j++)
		{
			nextAccelerations[j] = (nextVelocities[j] - currentVelocities[j])/timeIncrement;
			if (nextAccelerations[j] < accelLimits[j][0])
			{
				cout << "Acceleration limit of joint " << j << " exceeded. Velocity set to max value " << accelLimits[j][0] << "\n";
				nextAccelerations[j] = accelLimits[j][0];
			}
			else if (nextAccelerations[j] > accelLimits[j][1])
			{
				cout << "Acceleration limit of joint " << j << " exceeded. Velocity set to max value " << accelLimits[j][1] << "\n";
				nextAccelerations[j] = accelLimits[j][1];
			}
		}

		for (int j = 0; j < 4; j++)
		{
			currentVelocities[j] = nextVelocities[j];
		}

		MoveWithConfVelAcc(nextPosition, nextVelocities, nextAccelerations);

		Sleep(waitTime * 1000);
		outputFileTime += waitTime;
		waitTime = timeIncrement;
	}
	
		StopRobot();
		ResetRobot();
}

RobotPath::~RobotPath(void)
{
}
