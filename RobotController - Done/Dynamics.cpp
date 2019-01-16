#include "StdAfx.h"
#include "matrix.h"
#include "ensc-488.h"
#include <fstream>
#include<Windows.h>

using namespace std;

static void WriteConstantTorqueDataToFile(double** configurations, double* times, int count, string fileName)
{
	ofstream datafile;

	datafile.open(fileName);

	for (int i = 0; i < count; i++)
	{	
		datafile << times[i] << ", ";

		datafile << configurations[i][0] << ", " << configurations[i][1] << ", " << configurations[i][2] << ", " << configurations[i][3];

		datafile << "\n";
	}

}

static double* CalculateTorques12(double* angles, double* angularVel, double* angularAccel)
{
	Matrix<double>* Mass = new Matrix<double>(2, 2);				
	Matrix<double>* AccelMatrix = new Matrix<double>(2, 1);		    
	Matrix<double>* TorqueMatrix = new Matrix<double>(2, 1);	
	Matrix<double>* TorqueMatrixIntermed = new Matrix<double>(2, 1);// Coriolis and Centripetal Matrix
	Matrix<double>* InverseMass = new Matrix<double>(2, 2);			// Inverse Mass Matrix
	Matrix<double>* AngularMatrix = new Matrix<double>(2, 1);		// V matrix

	double M1 = 1.0; //M1 = M2 in our robot
	double M2 = 2.7; //M2 = M3+M4 in our robot
	double r = 0.03;

	double arm1 = .195;
	double arm2 = .142;

	Mass-> put(0, 0, (M2*(arm2*arm2)+2*M2*arm1*arm2*cos(angles[1])+(arm1*arm1)*(M1+M2)));
	Mass-> put(0, 1, (M2*(arm2*arm2) + arm1*arm2*M2*cos(angles[1])));
	Mass-> put(1, 0, (M2*(arm2*arm2) + M2*arm1*arm2*cos(angles[1])));
	Mass-> put(1, 1, (M2*(arm2*arm2)));
	
	AngularMatrix-> put(0, 0, (-M2*arm1*arm2*sin(angles[1])*(angularVel[1]*angularVel[1])-2*M2*arm1*arm2*sin(angles[1])*angularVel[0]*angularVel[1])); //V matrix
	AngularMatrix-> put(1, 0, (M2*arm1*arm2*sin(angles[1])*(angularVel[0]*angularVel[0])));

	AccelMatrix -> put(0,0, angularAccel[0]);
	AccelMatrix -> put(1,0, angularAccel[1]);

	*TorqueMatrixIntermed = (*Mass) * (*AccelMatrix);


	*TorqueMatrix = (*Mass) * (*AccelMatrix)+ *AngularMatrix; // calculate torques in this matrix

	double *torques12;
	torques12 = new double[2];
	torques12[0] = TorqueMatrix->get(0,0);
	torques12[1] = TorqueMatrix->get(1,0);

	return torques12; // return torques 1 and 2

}

static double* CalculateTorques34(double linearAccel, double angularAccel)
{
	double M2 = 2.7; //M2 = M3+M4 in our robot
	double M4 = 1.0;
	double G = -9.81;
	double r = 0.03;
	double *torques34;
	torques34 = new double[2];
	torques34[0] = (M2*G) + (M2*linearAccel);
	torques34[1] = (M4*r*r)*angularAccel;

	return torques34; // return torques 3 and 4

}

static double* CalculateAllTorques(double* positions, double* velocities, double* accelerations)
{
	double* allTorques = new double[4];

	double* torques12 = CalculateTorques12(positions, velocities, accelerations);
	double* torques34 = CalculateTorques34(accelerations[2], accelerations[3]);

	allTorques[0] = torques12[0];
	allTorques[1] = torques12[1];
	allTorques[2] = torques34[0];
	allTorques[3] = torques34[1];

	return allTorques;
}

static double** CalculateForwardDynamics12(double* JointPositions, double* JointVelocities, double *JointTorques, double time)
{
	Matrix<double> *Mass = new Matrix<double>(2, 2);					// Mass matrix
	double M1 = 1.0;
	double M2 = 2.7;
	double M3 = 2.7;
	double M4 = 1.0;
	double arm1 = 0.195;
	double arm2 = 0.142;
	
	Matrix<double> TorqueMatrix(2, 1);																				// Torque Matrix
	TorqueMatrix.put(0, 0, JointTorques[0]);
	TorqueMatrix.put(1, 0, JointTorques[1]);

	Matrix<double> accelerations(2, 1);
	Matrix<double> velocities(2, 1);
	Matrix<double> positions(2, 1);

	double ** behaviour = new double*[3];

	Matrix<double> SampleMass(2, 2);
	SampleMass.put(0, 0, ((M2*arm2) + (M2*2*arm1*arm2*cos(positions.get(1, 0))) + (arm1*arm1)*(M1 + M2)));		// Sample time Mass Matrix
	SampleMass.put(0, 1, (M2*arm2 + M2*arm1 + arm2));
	SampleMass.put(1, 0, (M2*(arm2*arm2) + M2*arm1*arm2*cos(positions.get(1, 0))));
	SampleMass.put(1, 1, (M2*(arm2*arm2)));

	Matrix<double> SampleInverseMass(2, 2);																						// Sample time Inverse Mass Matrix
	SampleInverseMass = SampleMass.getInverse();

	Matrix<double> SampleAngularMatrix(2, 1);																					// Sample time Coriolis and Centripetal Matrix
	SampleAngularMatrix.put(0, 0, -M2*arm1*arm2*sin(positions.get(1, 0))*(velocities.get(1, 0) * velocities.get(1, 0)) - 2 * M2*arm1*arm2*sin(positions.get(1, 0))*(velocities.get(0, 0)* velocities.get(1, 0)));
	SampleAngularMatrix.put(0, 0, M2*arm1*arm2*sin(positions.get(1, 0))*(velocities.get(0, 0) * velocities.get(0, 0)));

	accelerations = SampleInverseMass*(TorqueMatrix - SampleAngularMatrix);

	velocities.put(0, 0, JointVelocities[0] + (accelerations.get(0,0) * time));
	velocities.put(1, 0, JointVelocities[1] + (accelerations.get(1,0) * time));

	positions.put(0, 0, JointPositions[0] + (JointVelocities[0] * time) + (0.5 * accelerations.get(0,0) * (time*time)));
	positions.put(1, 0, JointPositions[1] + (JointVelocities[1] * time) + (0.5 * accelerations.get(1,0) * (time*time)));

	double * p = new double[2];
	double * v = new double[2];
	double * a = new double[2];

	p[0] = positions.get(0,0);
	p[1] = positions.get(1,0);

	v[0] = velocities.get(0,0);
	v[1] = velocities.get(1,0);

	a[0] = accelerations.get(0,0);
	a[1] = accelerations.get(1,0);

	behaviour[0] = p;
	behaviour[1] = v;
	behaviour[2] = a;

	return behaviour;
}

static double** CalculateForwardDynamics34(double* JointPositions, double* JointVelocities, double *JointTorques, double time)
{
	Matrix<double> *Mass = new Matrix<double>(2, 2);					// Mass matrix
	double M1 = 1.0;
	double M2 = 2.7;
	double M3 = 2.7;
	double M4 = 1.0;
	double arm1 = 0.195;
	double arm2 = 0.142;

	double gravity = 9.8;
	double radius4 = 0.03;
	Matrix<double> accelerations(2, 1);
	Matrix<double> velocities(2, 1);
	Matrix<double> positions(2, 1);

	double** behaviour = new double*[3];
	double *Calculated_Speeds = new double[6];

	accelerations.put(0, 0, (JointTorques[0] + M3*gravity) / M3);											// Joint 3 and 4 have constant acceleration
	accelerations.put(1, 0, (JointTorques[1]) / (((radius4*radius4))*M4));

	velocities.put(0, 0, JointVelocities[0] + (accelerations.get(0,0) * time));
	velocities.put(1, 0, JointVelocities[1] + (accelerations.get(1,0) * time));

	positions.put(0, 0, JointPositions[0] + (JointVelocities[0] * time) + (0.5 * accelerations.get(0,0) * (time*time)));
	positions.put(1, 0, JointPositions[1] + (JointVelocities[1] * time) + (0.5 * accelerations.get(1,0) * (time*time)));
	
	Calculated_Speeds[0] = positions.get(0, 0);
	Calculated_Speeds[1] = positions.get(1, 0);
	Calculated_Speeds[2] = velocities.get(0, 0);
	Calculated_Speeds[3] = velocities.get(1, 0);
	Calculated_Speeds[4] = accelerations.get(0, 0);
	Calculated_Speeds[5] = accelerations.get(1, 0);

	double * p = new double[2];
	double * v = new double[2];
	double * a = new double[2];

	p[0] = positions.get(0,0);
	p[1] = positions.get(1,0);

	v[0] = velocities.get(0,0);
	v[1] = velocities.get(1,0);
	
	a[0] = accelerations.get(0,0);
	a[1] = accelerations.get(1,0);

	behaviour[0] = p;
	behaviour[1] = v;
	behaviour[2] = a;

	return behaviour;
}

static double** CalculateForwardDynamics(double* positions, double* velocities, double* torques, double time)
{
	double* positions12 = new double[2];
	double* positions34 = new double[2];
	double* newPositions = new double[4];

	double* velocities12 = new double[2];
	double* velocities34 = new double[2];
	double* newVelocities = new double[4];

	double* newAccelerations = new double[4];

	double* torques12 = new double[2];
	double* torques34 = new double[2];

	double** behaviour12;
	double** behaviour34;

	double** behaviour = new double*[3];
	
	positions12[0] = positions[0];
	positions12[1] = positions[1];

	velocities12[0] = velocities[0];
	velocities12[1] = velocities[1];

	torques12[0] = torques[0];
	torques12[1] = torques[1];

	positions34[0] = positions[2];
	positions34[1] = positions[3];

	velocities34[0] = velocities[2];
	velocities34[1] = velocities[3];

	torques34[0] = torques[2];
	torques34[1] = torques[3];

	behaviour12 = CalculateForwardDynamics12(positions12, velocities12, torques12, time);

	behaviour34 = CalculateForwardDynamics34(positions34, velocities34, torques34, time);

	newPositions[0] = behaviour12[0][0];
	newPositions[1] = behaviour12[0][1];
	newPositions[2] = behaviour34[0][0];
	newPositions[3] = behaviour34[0][1];

	newVelocities[0] = behaviour12[1][0];
	newVelocities[1] = behaviour12[1][1];
	newVelocities[2] = behaviour34[1][0];
	newVelocities[3] = behaviour34[1][1];

	newAccelerations[0] = behaviour12[2][0];
	newAccelerations[1] = behaviour12[2][1];
	newAccelerations[2] = behaviour34[2][0];
	newAccelerations[3] = behaviour34[2][1];

	behaviour[0] = newPositions;
	behaviour[1] = newVelocities;
	behaviour[2] = newAccelerations;

	return behaviour;
}

static double* CoerceAllJointPositionsInRange(double* positions)
{
	double maxValues[4] = {150, 100, -100, 160};
	double minValues[4] = {-150, -200. -200, -160};

	double* newPositions = new double[4];

	for (int i = 0; i < 4; i++)
	{
		if(positions[i] > maxValues[i])
		{
			newPositions[i] = maxValues[i];
		}
		else if (positions[i] < minValues[i])
		{
			newPositions[i] = minValues[i];
		}
		else
		{
			newPositions[i] = positions[i];
		}
	}

	return newPositions;
}


static void MoveWithConstantTorque(double* torques, double totalTime, string fileName)
{
	double timeIncrement = 0.02;
	double time = 0;
	int configurationCount = static_cast<int>(ceil(totalTime/timeIncrement));
	double** behaviour;
	double* a;
	double** executedPositions = new double*[configurationCount - 1];
	double* times = new double[configurationCount - 1];
	double currentConfiguration[4];

	GetConfiguration(currentConfiguration);
	
	double* p = new double[4];
	p[0] = currentConfiguration[0];
	p[1] = currentConfiguration[1];
	p[2] = currentConfiguration[2];
	p[3] = currentConfiguration[3];

	double* v = new double[4];
	v[0] = 0;
	v[1] = 0;
	v[2] = 0;
	v[3] = 0;

	for (int i = 0; i < configurationCount-1; i++)
	{
		behaviour = CalculateForwardDynamics(p, v, torques, timeIncrement);
		
		p[0] = behaviour[0][0];
		p[1] = behaviour[0][1];
		p[2] = behaviour[0][2];
		p[3] = behaviour[0][3];
		v[0] = behaviour[1][0];
		v[1] = behaviour[1][1];
		v[2] = behaviour[1][2];
		v[3] = behaviour[1][3];

		p = CoerceAllJointPositionsInRange(p);
		a = behaviour[2];
		
		double* configuration = new double[4];
		
		for (int j = 0; j < 4; j++)
		{
			currentConfiguration[j] = p[j];
			configuration[j] = p[j];
		}

		time += timeIncrement;
		times[i] = time;
		executedPositions[i] = configuration;

		DisplayConfiguration(currentConfiguration);
		Sleep(timeIncrement*1000);
	}

	WriteConstantTorqueDataToFile(executedPositions, times, configurationCount - 1, fileName);
}


