#pragma once
#include<iostream>
using namespace std;

class InverseKinResult
{
public:
	string message;
	double ** Configurations;
	int SolutionCount;
	InverseKinResult(void);
	void SetResults(double ** results, int num);
	void SetMessage(string newMessage);
	~InverseKinResult(void);
};

