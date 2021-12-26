#ifndef MPC_H
#define MPC_H

// MPC
#include <vector>
#include <iostream>
#include <cmath>
#include <time.h>

#include <pybind11/pybind11.h>
//#include <pybind11/embed.h>
#include <pybind11/stl.h>
//#include <pybind11/numpy.h>

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

    float x_vel_ref;
    float x_pos_ref;

    float y_vel_ref;
    float y_pos_ref;

    float z_vel_ref;
    float z_pos_ref;
    

    float x_vel_demand;
    float x_pos_demand;

    float y_vel_demand;
    float y_pos_demand;

    float z_vel_demand;
    float z_pos_demand;

    void mpcOperation(float v_ref=0, float p_ref=100, float v_init=0, float p_init=0, int v_input_begin=0);
    void mpcOperation(std::vector<float> v_ref, std::vector<float> p_ref, std::vector<float> v_d, std::vector<float> p_d, std::vector<float> v_input);


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