#include "timestepper.h"

#include <cstdio>

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   std::vector<Vector3f> currentState = particleSystem->getState();
   std::vector<Vector3f> change = particleSystem->evalF(currentState);
   std::vector<Vector3f> resultingState = change;
   for (int i = 0; i < resultingState.size(); ++i) {
      resultingState[i] = currentState[i] + stepSize * change[i];
   } 
   // set system state
   particleSystem->setState(resultingState);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   std::vector<Vector3f> currentState = particleSystem->getState();
   std::vector<Vector3f> f_0 = particleSystem->evalF(currentState);
   std::vector<Vector3f> forwardEulerVector = f_0;
   for (int i = 0; i < forwardEulerVector.size(); ++i) {
      forwardEulerVector[i] = currentState[i] + stepSize * f_0[i];
   } 
   std::vector<Vector3f> f_1 = particleSystem->evalF(forwardEulerVector);
   std::vector<Vector3f> resultingState = f_0;
   for (int i = 0; i < resultingState.size(); ++i) {
      resultingState[i] = currentState[i] + stepSize/2 * (f_0[i] + f_1[i]);
   } 
   // set system state
   particleSystem->setState(resultingState);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   std::vector<Vector3f> currentState = particleSystem->getState();
   std::vector<Vector3f> k_1 = particleSystem->evalF(currentState);
   std::vector<Vector3f> k_2;
   for (int i = 0; i < k_1.size(); ++i) {
      k_2.push_back(currentState[i] + (stepSize/2 * k_1[i]));
   } 
   k_2 = particleSystem->evalF(k_2);
   std::vector<Vector3f> k_3;
   for (int i = 0; i < k_2.size(); ++i) {
      k_3.push_back(currentState[i] + (stepSize/2 * k_2[i]));
   } 
   k_3 = particleSystem->evalF(k_3);
   std::vector<Vector3f> k_4;
   for (int i = 0; i < k_3.size(); ++i) {
      k_4.push_back(currentState[i] + (stepSize * k_3[i]));
   }
   k_4 = particleSystem->evalF(k_4);
   std::vector<Vector3f> resultingState;
   for (int i = 0; i < k_1.size(); ++i) {
      resultingState.push_back(currentState[i] + (stepSize/6 * (k_1[i] + 2*k_2[i] + 2*k_3[i] + k_4[i])));
   }
   // set system state
   particleSystem->setState(resultingState);
}

