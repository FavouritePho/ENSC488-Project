#include "stdafx.h"

#include"stateid.h"
#include <string>
#include <conio.h>
#include <iostream>
#include"Console.h"
#include "Dynamics.cpp"


using namespace std;

Robot * robot;
double jointParams[4];
double xyz[4];
char coordinateCommand = '\0';


	double min1 = -150;
	double max1 = 150;
	double min2 = -100;
	double max2 = 100;
	double min3 = -200;
	double max3 = -100;
	double min4 = -160;
	double max4 = 160;

void HandleCommand(char command)
{
	if (command == 'O' || command == 'o')
	{
		OpenMonitor();
		cout << "Opening monitor. \n";
	}
	if (command == 'C' || command == 'c')
	{
		CloseMonitor();
		cout << "Closing monitor. \n";
	}
	if (command == 'M' || command == 'm')
	{
		HandleMove();
	}
	if (command == 'S' || command == 's')
	{
		cout << "Stopping robot. Must reset (command A) before robot can be moved. \n";
		StopRobot();
	}
	if (command == 'A' || command == 'a')
	{
		cout << "Reseting robot. \n";
		ResetRobot();
	}
	if (command == 'G' || command == 'g')
	{
		cout << "Closing gripper. \n";
		Grasp(true);
	}
	if (command == 'F' || command == 'f')
	{
		cout << "Opening gripper. \n";
		Grasp(false);
	}
	if (command == 'R' || command == 'r')
	{
		HandleTrajectory();	
	}
	if (command == 'T' || command == 't')
	{
		cout << "Running Test. \n";
		TestPathGenerator();
	}
	if (command == 'P' || command == 'p')
	{
		cout << "Planning a trajectory. \n";
		HandleTrajectory();
	}
	if (command == 'B' || command == 'b')
	{
		cout << "Doing forward dynamics. \n";
		HandleForwardDynamics();
	}
	if (command == 'I' || command == 'i')
	{
		cout << "Planning and following a trajectory using a controller. \n";
		HandleInverseDynamics();
	}
}

void HandleForwardDynamics()
{
	double * torques = new double[4];
	double time;
	string fileName;
	
	cout << "Enter torques to be applied to each joint  (t1, t2, f3, t4)  \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> torques[i];
	}

	if (!CheckIfTorquesAreInRange(torques))
	{
		return;
	}

	cout << "Enter time for which torque should be applied\n";

	cin >> time;

	cout << "Enter filename to write data to \n";

	cin >> fileName;

	MoveWithConstantTorque(torques, time, fileName);
}

bool CheckIfTorquesAreInRange(double* torques)
{
	double maxRotaryTorque = 16;
	double maxLinearForce = 45;

	for (int i = 0; i < 4; i++)
	{
		if (i==2)
		{
			continue;
		}
		if (abs(torques[i]) > maxRotaryTorque)
		{
			cout << "Torque for joint " << i + 1 << " is out of range, aborting move. \n";
			return false;
		}
	}

	if (abs(torques[2]) > maxLinearForce)
	{
		cout << "Force for joint 3 is out of range, aborting move. \n";
		return false;
	}

	return true;
}

void HandleInverseDynamics()
{
	double * goalFrame = new double[4];
	double * viaFrame1 = new double[4];
	double * viaFrame2 = new double[4];
	double * viaFrame3 = new double[4];
	double ** allFrames = new double*[4];
	string fileName;
	double totalTime;
	
	cout << "Please enter goal frame in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> goalFrame[i];
	}
	
	cout << "Please enter via frame 1 in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> viaFrame1[i];
	}
	
	cout << "Please enter via frame 2 in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> viaFrame2[i];
	}
	
	cout << "Please enter via frame 3 in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> viaFrame3[i];
	}

	cout << "Please enter total time for trajectory \n";

	cin >> totalTime;

	cout << "Please enter filename for output plots \n";

	cin >> fileName;

	allFrames[0] = viaFrame1;
	allFrames[1] = viaFrame2;
	allFrames[2] = viaFrame3;
	allFrames[3] = goalFrame;
	
	RobotPath * path = new RobotPath(allFrames, totalTime, 5);

	cout << "\n" << "Moving through joint configurations: ";

	for (int i = 0; i < 5; i++)
	{
		cout << "theta1 = " << path -> JointConfigs[i][0] << ", theta2 = " << path -> JointConfigs[i][1];
		cout << ", d3 = " << path -> JointConfigs[i][2] << ", theta4 = " << path -> JointConfigs[i][3] << "\n";
	}

	cout << "Ok? Y/N \n";

	char a = ' ';
	cin >> a;

	if (a == 'Y')
	{
		cout << "Executing path \n";
		path -> ExecutePathUsingController(fileName);
	}
	else
	{
		cout << "Aborting move \n";
		return;
	}
}

void HandleTrajectory()
{
	double * goalFrame = new double[4];
	double * viaFrame1 = new double[4];
	double * viaFrame2 = new double[4];
	double * viaFrame3 = new double[4];
	double ** allFrames = new double*[4];
	double totalTime;
	string filename;
	
	cout << "Please enter goal frame in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> goalFrame[i];
	}
	
	cout << "Please enter via frame 1 in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> viaFrame1[i];
	}
	
	cout << "Please enter via frame 2 in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> viaFrame2[i];
	}
	
	cout << "Please enter via frame 3 in cartesian coordinates (x,y,z,phi) \n";
	
	for (int i = 0; i < 4; i++)
	{
		cin >> viaFrame3[i];
	}

	cout << "Please enter total time for trajectory \n";

	cin >> totalTime;

	allFrames[0] = viaFrame1;
	allFrames[1] = viaFrame2;
	allFrames[2] = viaFrame3;
	allFrames[3] = goalFrame;
	
	RobotPath * path = new RobotPath(allFrames, totalTime, 5);

	cout << "\n" << "Moving through joint configurations: ";

	for (int i = 0; i < 5; i++)
	{
		cout << "theta1 = " << path -> JointConfigs[i][0] << ", theta2 = " << path -> JointConfigs[i][1];
		cout << ", d3 = " << path -> JointConfigs[i][2] << ", theta4 = " << path -> JointConfigs[i][3] << "\n";
	}

	cout << "Enter filename for output data \n";

	cin >> filename;

	cout << "Ok? Y/N \n";

	char a = ' ';
	cin >> a;

	if (a == 'Y' || 'y')
	{
		cout << "Executing path \n";
		path -> ExecutePath(filename);
	}
	else
	{
		cout << "Aborting move \n";
		return;
	}

}

void TestPathGenerator()
{
	double ** cartesianConfigs = new double*[4];

	double * p1 = new double[4];
	double * p2 = new double[4];
	double * p3 = new double[4];
	double * p4 = new double[4];
	double totalTime = 10;
	
	p1[0] = 337;
	p1[1] = 0;
	p1[2] = 100;
	p1[3] = 0;

	p2[0] = 300;
	p2[1] = 10;
	p2[2] = 80;
	p2[3] = 10;

	p3[0] = 250;
	p3[1] = 50;
	p3[2] = 60;
	p3[3] = 20;

	p4[0] = 120;
	p4[1] = 100;
	p4[2] = 40;
	p4[3] = 30;

	cartesianConfigs[0] = p1;
	cartesianConfigs[1] = p2;
	cartesianConfigs[2] = p3;
	cartesianConfigs[3] = p4;

	RobotPath * path = new RobotPath(cartesianConfigs, totalTime, 5);

	path -> ExecutePath("test");
	return;
}

void HandleMove()
{
	coordinateCommand = '\0';
	cout << "Enter X for XYZ coordinates and Phi, enter J for joint parameters \n";
	cin >> coordinateCommand;

	if (coordinateCommand == 'X' || coordinateCommand == 'x')
	{
		cout << "Enter desired position (x, y, z, phi) \n";
		cin >> xyz[0];
		cin >> xyz[1];
		cin >> xyz[2];
		cin >> xyz[3];

		InverseKinResult * result = robot -> InverseKin(xyz[0], xyz[1], xyz[2], xyz[3]);
		
		double currentPosition[4];
		GetConfiguration(currentPosition);
		
		double * mySolution =  robot -> ClosestSolution(result -> Configurations, currentPosition, result -> SolutionCount);

		cout << result -> message << "\n";
		
		double InverseJointParam[4] = { mySolution[0], mySolution[1], mySolution[2], mySolution[3]};
		
		if (!CheckIfJointParamsInRange(InverseJointParam))
		{
			cout << "Joint parameters out of range. Aborting move. \n";
			return;
		}
		
		cout << "\n" << "Move to position? Y/N \n";

		cin >> coordinateCommand;

		if (coordinateCommand == 'Y' || coordinateCommand == 'y')
		{
			MoveToConfiguration(InverseJointParam, false);
		}
		else
		{
			cout << "Aborting move \n";
		}

	}
	else if (coordinateCommand == 'J' || coordinateCommand == 'j' )
	{
		cout << "Enter desired joint parameters (t1, t2, d3, t4)\n";
		// todo - calculate position (forward kin) and display
		
		cin >> jointParams[0];
		cin >> jointParams[1];
		cin >> jointParams[2];
		cin >> jointParams[3];

		if (!CheckIfJointParamsInRange(jointParams))
		{
			cout << "Joint parameters out of range. Aborting move. \n";
			return;
		}

		double * xyz;
		xyz = robot -> ForwardKin(jointParams[0], jointParams[1] , jointParams[2], jointParams[3]);

		cout << "Moving to position x = " << xyz[0] << ", y = " << xyz[1] << ", z = " << xyz[2] << ", phi = " << xyz[3];
		cout << "\n" << "Position okay? Y/N \n";

		cin >> coordinateCommand;

		if (coordinateCommand == 'Y' || coordinateCommand == 'y')
		{
			MoveToConfiguration(jointParams, false);
		}
		else
		{
			cout << "Aborting move \n";
		}
	}
	else
	{
		cout << "Invalid command. Aborting move. \n";
	}
}

bool CheckIfJointParamsInRange(double * params)
{
	bool inRange = true;
	string message = "\n";

	if (params[0] <  min1 || params[0] > max1)
	{
		inRange = false;
		message += "Parameter 1 is out of range. \n";
	}

	if (params[1] < min2 || params[1] > max2)
	{
		inRange = false;
		message += "Parameter 2 is out of range. \n";
	}
	
	if (params[2] <min3 || params[2] > max3)
	{
		inRange = false;
		message += "Parameter 3 is out of range. \n";
	}
	
	if (params[3] < min4 || params[3]  >max4)
	{
		inRange = false;
		message += "Parameter 3 is out of range. \n";
	}

	if(!inRange)
	{
		cout << message;
		cout << "Acceptable range: \n";
		cout << "Parameter 1: " <<  min1 << " - " << max1 << "\n";
		cout << "Parameter 2: " <<  min2 << " - " <<  max2 << "\n";
		cout << "Parameter 3: " << min3 << " - " << max3 << "\n";
		cout << "Parameter 4: " <<  min4 << " - " << max4 << "\n";
	}

	return inRange;
}

void BuildRobot()
{
	robot = new Robot();
}