#include "stdafx.h"

#include"ensc-488.h"

#include"stateid.h"

#include <conio.h>
#include <iostream>
#include"Robot.h"

using namespace std;

double jointParams[4];
double xyz[3];
char coordinateCommand = '\0';

void HandleMove()
{
	coordinateCommand = '\0';
	cout << "Enter X for XYZ coordinates, enter J for joint parameters \n";
	cin >> coordinateCommand;

	if (coordinateCommand == 'X')
	{
		cout << "Enter desired position \n";
		cin >> xyz[0];
		cin >> xyz[1];
		cin >> xyz[2];

		// todo - calculate joint parameters (inverse kin)
		}
	else if (coordinateCommand == 'J')
	{
		cout << "Enter desired joint parameters \n";
		// todo - calculate position (forward kin) and display
		
		cin >> jointParams[0];
		cin >> jointParams[1];
		cin >> jointParams[2];
		cin >> jointParams[3];

		
			
		MoveToConfiguration(jointParams, false);
	}
	else
	{
		cout << "Invalid command. Aborting move. \n";
	}
}