// RobotController.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include"ensc-488.h"

#include"stateid.h"

#include <conio.h>
#include <iostream>
#include"Console.h"
#include"Robot.h"
#include<Windows.h>

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	cout << "Initializing robot \n";
	if (ResetRobot())
	{
		cout << "Robot initialized. Ready to use. \n";
		cout << "\nEnter a command: \nO = Open Monitor \nC = Close Monitor \nM = Move Arm \nS = Stop Arm \nA = Reset Robot \nG = Close Gripper \nF = Open Gripper  \nP = Plan a Trajectory \n \n";
	}
	else
	{
		cout << "Failed to initizlize robot. Aborting program \n";
		Sleep(5000);
		return 0;
	}
	char command;
	
	while (true)
	{
		cin >> command;
		HandleCommand(command);
	}
	return 0;
}


