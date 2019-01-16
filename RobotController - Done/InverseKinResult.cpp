#include "StdAfx.h"
#include "InverseKinResult.h"
#include<iostream>
using namespace std;



InverseKinResult::InverseKinResult(void)
{
}


InverseKinResult::~InverseKinResult(void)
{
}

void InverseKinResult::SetResults(double ** results, int num)
{
	Configurations = results;
	SolutionCount = num;
}

void InverseKinResult::SetMessage(string newMessage)
{
	message = newMessage;
}
