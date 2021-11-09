#ifndef MPC_H
#define MPC_H

// MPC
#include <vector>
#include <iostream>
#include <cmath>
#include <time.h>

#define PI 3.14159265




class MPC{

private:

    // MPC
    std::vector<float> selfDefPosDemand;
    std::vector<float> selfDefVelDemand;

    float amplitude = 100;
    float period = 10;


    void initDemand();
    

public:

    float x_vel_demand;
    float x_pos_demand;

    float y_vel_demand;
    float y_pos_demand;

    float z_vel_demand;
    float z_pos_demand;

    float sinePosDemand(float time);
    float cosVelDemand(float time);

    float stepPosDemand(float time);

    float sineToTenPosDemand(float time);
    float cosToTenVelDemand(float time);

    float powThreePosDemand(float time);
    float powTwoVelDemand(float time);

    MPC();




};





#endif