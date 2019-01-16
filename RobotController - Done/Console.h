#include"Robot.h"

void HandleCommand(char command);
void HandleMove();
bool CheckIfJointParamsInRange(double * params);
void TestPathGenerator();
void HandleTrajectory();
void HandleForwardDynamics();
void HandleInverseDynamics();
bool CheckIfTorquesAreInRange(double* torques);