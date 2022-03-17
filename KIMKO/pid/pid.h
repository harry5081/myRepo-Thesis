#ifndef PID_H
#define PID_H

// MPC
#include <vector>
#include <iostream>
#include <cmath>
#include <time.h>

#define PI 3.14159265

void initDemand();
float sinePosDemand(float time);
float cosVelDemand(float time);

float stepPosDemand(float time);

float sineToTenPosDemand(float time);
float cosToTenVelDemand(float time);

//PID
class PID{

private:
    float kp=0;
    float ki=0;
    //float Ki=0.0;
    

    float kf=0;
    float kd=0;

    float timeBuffer=0;
    float errorAccumulate=0;


public:
    float posDemand;
    //int vInput;
    float posSenValue;

    PID(float Kp_set, float Ki_set, float Kf_set, float Kd_set);
  
    int pidExe(float posError, int velDemand, float velError);
    int pidExeAngle(float posError, int velDemand, float velError);

};






#endif