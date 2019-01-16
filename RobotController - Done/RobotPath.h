#pragma once
#include "Arm.h"
#include "Robot.h"
#include "InverseKinResult.h"
#include "CubicSpline.h"
#include <conio.h>
#include <fstream>
#include <string>
#include<Windows.h>


class RobotPath
{
public:
	RobotPath(double ** cartesianConfigs, double totalTimes, int numConfigs);
	void SetVelocityAndAccelerationLimits();
	void CalculateJointConfigs();
	void CalculateSplines();
	double * CalculateTimes();
	// void DumpData(stringFileName);
	void ExecutePath(string fileName);
	// void CalculateError(string FileName);
	void ExecutePathUsingController(string fileName);
	void WriteDataToFile(double** plannedTrajectory, double** executedTrajectory, double** plannedVelocity, double** executedVelocity, double** plannedAcceleration, double** executedAcceleration, double** executedTorques, double** positionError, double** velocityError, double* times, string fileName, int count);
	double* CoerceControllerPositionsInRange(double* Position);
	double* CoerceControllerTorquesInRange(double* torques);
	double TotalTime;
	double ** JointConfigs;
	double ** CartesianConfigs;
	double * Theta1s;
	double * Theta2s;
	double * D3s;
	double * Theta4s;
	double * Times;
	double ** velocityLimits;
	double ** accelLimits;
	int PositionCount;
	CubicSpline * Theta1Spline;
	CubicSpline * Theta2Spline;
	CubicSpline * D3Spline;
	CubicSpline * Theta4Spline;
	~RobotPath(void);
};

