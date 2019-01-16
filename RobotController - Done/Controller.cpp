#include "StdAfx.h"
#include "RobotPath.h"
#include <time.h>
#include "ensc-488.h"
#include "Dynamics.cpp"

void RobotPath::ExecutePathUsingController(string fileName)
{
	double kp[4] = {175,110,40,20};
	double kv[4] = {26.458, 20.976,12.649,8.944};
	
	int storageResolution = 5;
	double time = 0;
	double timeIncrement = 0.02;
	int configurationCount = static_cast<int>(ceil(TotalTime/timeIncrement));
	int storageSize = configurationCount/storageResolution;

	double positionToDisplay[4];
	double* positionErrors = new double[4];
	double* velocityErrors = new double[4];
	double ** currentState = new double*[3];
	double desiredVelocities[4];
	double desiredAccelerations[4];
	double desiredPositions[4];
	double* inRangePositions;
	double * torques;
	double** executedTorques = new double*[storageSize];
	double** executedTrajectory = new double*[storageSize];
	double** allPositionErrors = new double*[storageSize];
	double** allVelocityErrors = new double*[storageSize];
	double** plannedTrajectory = new double*[storageSize];
	double** plannedVelocities = new double*[storageSize];
	double** executedVelocities = new double*[storageSize];
	double** plannedAccelerations = new double*[storageSize];
	double** executedAccelerations = new double*[storageSize];

	double* times = new double[storageSize];
	int a = 0;

	for (int j = 0; j < 4; j++)
	{
		positionErrors[j] = 0;
		velocityErrors[j] = 0;
	}

	desiredPositions[0] = Theta1Spline -> GetYAtX(0);
	desiredPositions[1] = Theta2Spline -> GetYAtX(0);
	desiredPositions[2] = Theta4Spline -> GetYAtX(0);
	desiredPositions[3] = D3Spline -> GetYAtX(0);

	desiredVelocities[0] = Theta1Spline -> GetVelocityAtX(0);
	desiredVelocities[1] = Theta2Spline -> GetVelocityAtX(0);
	desiredVelocities[2] = Theta4Spline -> GetVelocityAtX(0);
	desiredVelocities[3] = D3Spline -> GetVelocityAtX(0);

	currentState[0] = desiredPositions;
	currentState[1] = desiredVelocities;

	positionToDisplay[0] = Theta1Spline -> GetYAtX(0);
	positionToDisplay[1] = Theta2Spline -> GetYAtX(0);
	positionToDisplay[2] = Theta4Spline -> GetYAtX(0);
	positionToDisplay[3] = D3Spline -> GetYAtX(0);

	DisplayConfiguration(positionToDisplay);

	for (int i = 0; i < configurationCount; i++)
	{
		desiredAccelerations[0] = Theta1Spline -> GetAccelerationAtX(time);
		desiredAccelerations[1] = Theta2Spline -> GetAccelerationAtX(time);
		desiredAccelerations[2] = Theta4Spline -> GetAccelerationAtX(time);
		desiredAccelerations[3] = D3Spline -> GetAccelerationAtX(time);

		for (int j = 0; j < 4; j++)
		{
			desiredAccelerations[j] += (positionErrors[j] * kp[j] + velocityErrors[j] * kv[j]);
		}

		torques = CalculateAllTorques(currentState[0], currentState[1], desiredAccelerations);

		torques = CoerceControllerTorquesInRange(torques);

		currentState = CalculateForwardDynamics(desiredPositions, desiredVelocities, torques, timeIncrement);
		
		inRangePositions = CoerceControllerPositionsInRange(currentState[0]);

		for (int j = 0; j < 4; j++)
		{
			positionToDisplay[j] = currentState[0][j];
		}

		DisplayConfiguration(positionToDisplay);

		desiredPositions[0] = Theta1Spline -> GetYAtX(time + timeIncrement);
		desiredPositions[1] = Theta2Spline -> GetYAtX(time + timeIncrement);
		desiredPositions[2] = Theta4Spline -> GetYAtX(time + timeIncrement);
		desiredPositions[3] = D3Spline -> GetYAtX(time + timeIncrement);

		desiredVelocities[0] = Theta1Spline -> GetVelocityAtX(time + timeIncrement);
		desiredVelocities[1] = Theta2Spline -> GetVelocityAtX(time + timeIncrement);
		desiredVelocities[2] = Theta4Spline -> GetVelocityAtX(time + timeIncrement);
		desiredVelocities[3] = D3Spline -> GetVelocityAtX(time + timeIncrement);

		for (int j = 0; j < 4; j++)
		{
			positionErrors[j] = desiredPositions[j] - currentState[0][j]; 
			velocityErrors[j] = desiredVelocities[j] - currentState[1][j];
		}

		if ((i % storageResolution) == 0)
		{
			double * errorP = new double[4];
			double * errorV = new double[4];
			double * executedPosition = new double[4];
			double * plannedPosition = new double[4];
			double * executedTorque = new double[4];
			double * plannedVelocity = new double[4];
			double * executedVelocity = new double[4];
			double * plannedAcceleration = new double[4];
			double * executedAcceleration = new double[4];

			for (int j = 0; j < 4; j++)
			{
				errorP[j] = positionErrors[j];
				errorV[j] = velocityErrors[j];
				plannedPosition[j] = desiredPositions[j];
				executedPosition[j] = currentState[0][j];
				plannedVelocity[j] = desiredVelocities[j];
				executedVelocity[j] = currentState[1][j];
				executedAcceleration[j] = currentState[2][j];
				executedTorque[j] = torques[j];
			}

			plannedAcceleration[0] = Theta1Spline -> GetAccelerationAtX(time + timeIncrement);
			plannedAcceleration[1] = Theta2Spline -> GetAccelerationAtX(time + timeIncrement);
			plannedAcceleration[2] = Theta4Spline -> GetAccelerationAtX(time + timeIncrement);
			plannedAcceleration[3] = D3Spline -> GetAccelerationAtX(time + timeIncrement);
			
			executedTorques[a] = executedTorque;
			plannedTrajectory[a] = plannedPosition;
			executedTrajectory[a] = executedPosition;
			plannedVelocities[a] = plannedVelocity;
			executedVelocities[a] = executedVelocity;
			plannedAccelerations[a] = plannedAcceleration;
			executedAccelerations[a] = executedAcceleration;
			allPositionErrors[a] = errorP;
			allVelocityErrors[a] = errorV;

			times[a] = time;
			a++;
		}

		Sleep(timeIncrement * 1000);
		time += timeIncrement;
	}
	
		StopRobot();
		ResetRobot();

		WriteDataToFile(plannedTrajectory, executedTrajectory, plannedVelocities, executedVelocities, plannedAccelerations, executedAccelerations, executedTorques, allPositionErrors, allVelocityErrors, times, fileName, storageSize);
}

double* RobotPath::CoerceControllerPositionsInRange(double* positions)
{
	double maxValues[4] = {150, 100, 200, 160};

	double* newPositions = new double[4];

	for (int i = 0; i < 4; i++)
	{
		if(positions[i] > maxValues[i])
		{
			newPositions[i] = maxValues[i];
		}
		else if (positions[i] < -maxValues[i])
		{
			newPositions[i] = -maxValues[i];
		}
		else
		{
			newPositions[i] = positions[i];
		}
	}

	return newPositions;
}

double* RobotPath::CoerceControllerTorquesInRange(double* torques)
{
	double maxRotaryTorque = 16;
	double maxLinearForce = 45;
	double* newTorques = new double[4];

	for(int i = 0; i < 4; i++)
	{
		if (i == 2)
		{
			continue;
		}
		if (torques[i] > maxRotaryTorque)
		{
			newTorques[i] = maxRotaryTorque;
			cout << "Torque for joint" << i + 1 << " is out of range. Setting to max value " << maxRotaryTorque << "\n";
		}
		else if (torques[i] < (-maxRotaryTorque))
		{
			newTorques[i] = -maxRotaryTorque;
			cout << "Torque for joint" << i + 1 << " is out of range. Setting to max value " << maxRotaryTorque << "\n";
		}
		else
		{
			newTorques[i] = torques[i];
		}
	}

	if (torques[2] > maxLinearForce)
	{
		newTorques[2] = maxLinearForce;
		cout << "Torque for joint 3 is out of range. Setting to max value " << maxRotaryTorque << "\n";
	}
	else if (torques[2] < (-maxLinearForce))
	{
		newTorques[2] = -maxLinearForce;
		cout << "Torque for joint 3 is out of range. Setting to max value " << maxRotaryTorque << "\n";
	}
	else
	{
		newTorques[2] = torques[2];
	}

	return newTorques;

}

void RobotPath::WriteDataToFile(double** plannedTrajectory, double** executedTrajectory, double** plannedVelocity, double** executedVelocity, double** plannedAcceleration, double** executedAcceleration, double** executedTorques, double** positionError, double** velocityError, double* times, string fileName, int count)
{
	string plannedTrajectoryFileName = fileName + "PlannedTrajectory.csv";
	string executedTrajectoryFileName = fileName + "ExecutedTrajectory.csv";
	string executedTorqueFileName = fileName + "ExecutedTorque.csv";
	string positionErrorFileName = fileName + "PositionError.csv";
	string velocityErrorFileName = fileName + "VelocityError.csv";
	string plannedVelocitiesFileName = fileName + "PlannedVelocities.csv";
	string executedVelocitiesFileName = fileName + "ExecutedVelocities.csv";
	string plannedAccelerationsFileName = fileName + "PlannedAccelerations.csv";
	string executedAccelerationsFileName = fileName + "ExecutedAccelerations.csv";

	ofstream plannedTrajectoryDataFile;
	ofstream executedTrajectoryDataFile;
	ofstream executedTorqueDataFile;
	ofstream positionErrorDataFile;
	ofstream velocityErrorDataFile;
	ofstream plannedVelocitiesDataFile;
	ofstream executedVelocitiesDataFile;
	ofstream plannedAccelerationsDataFile;
	ofstream executedAccelerationsDataFile;

	plannedTrajectoryDataFile.open(plannedTrajectoryFileName);
	executedTrajectoryDataFile.open(executedTrajectoryFileName);
	executedTorqueDataFile.open(executedTorqueFileName);
	positionErrorDataFile.open(positionErrorFileName);
	velocityErrorDataFile.open(velocityErrorFileName);
	plannedVelocitiesDataFile.open(plannedVelocitiesFileName);
	executedVelocitiesDataFile.open(executedVelocitiesFileName);
	plannedAccelerationsDataFile.open(plannedAccelerationsFileName);
	executedAccelerationsDataFile.open(executedAccelerationsFileName);

	for (int i = 0; i < count - 1; i++)
	{	
		plannedTrajectoryDataFile << times[i] << ", ";
		executedTrajectoryDataFile << times[i] << ", ";
		executedTorqueDataFile << times[i] << ", ";
		positionErrorDataFile << times[i] << ", ";
		velocityErrorDataFile << times[i] << ", ";
		plannedVelocitiesDataFile  << times[i] << ", ";
		executedVelocitiesDataFile << times[i] << ", ";
		plannedAccelerationsDataFile << times[i] << ", ";
		executedAccelerationsDataFile << times[i] << ", ";

		plannedTrajectoryDataFile << plannedTrajectory[i][0] << ", " << plannedTrajectory[i][1] << ", " << plannedTrajectory[i][2] << ", " << plannedTrajectory[i][3] << "\n";

		executedTrajectoryDataFile << executedTrajectory[i][0] << ", " << executedTrajectory[i][1] << ", " << executedTrajectory[i][2] << ", " << executedTrajectory[i][3] << "\n";

		executedTorqueDataFile << executedTorques[i][0] << ", " << executedTorques[i][1] << ", " << executedTorques[i][2] << ", " << executedTorques[i][3] << "\n";

		positionErrorDataFile << positionError[i][0] << ", " << positionError[i][1] << ", " << positionError[i][2] << ", " << positionError[i][3] << "\n";

		velocityErrorDataFile << velocityError[i][0] << ", " << velocityError[i][1] << ", " << velocityError[i][2] << ", " << velocityError[i][3] << "\n";

		plannedVelocitiesDataFile << plannedVelocity[i][0] << ", " << plannedVelocity[i][1] << ", " << plannedVelocity[i][2] << ", " << plannedVelocity[i][3] << "\n";

		executedVelocitiesDataFile << executedVelocity[i][0] << ", " << executedVelocity[i][1] << ", " << executedVelocity[i][2] << ", " << executedVelocity[i][3] << "\n";

		plannedAccelerationsDataFile << plannedAcceleration[i][0] << ", " << plannedAcceleration[i][1] << ", " << plannedAcceleration[i][2] << ", " << plannedAcceleration[i][3] << "\n";
			
		executedAccelerationsDataFile << executedAcceleration[i][0] << ", " << executedAcceleration[i][1] << ", " << executedAcceleration[i][2] << ", " << executedAcceleration[i][3] << "\n";
	}
}
