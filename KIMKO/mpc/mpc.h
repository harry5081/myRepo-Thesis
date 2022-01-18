#ifndef MPC_H
#define MPC_H

// MPC
#include <vector>
#include <iostream>
#include <cmath>
#include <time.h>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
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

    float pre_vd = 0;
    float pre_pd = 0;

    std::vector<float> pre_vd_pd ={0,0,0,0,0,0}; // pre_vdx, pre_pdx, pre_vdy, pre_pdy, pre_vdz, pre_pdz, 

    
    

public:

    float x_vel_ref=0;
    float x_pos_ref=-300;

    float y_vel_ref=0;
    float y_pos_ref=200;

    float z_vel_ref=0;
    float z_pos_ref=-45;
    

    float x_vel_demand;
    float x_pos_demand;

    float y_vel_demand;
    float y_pos_demand;

    float z_vel_demand;
    float z_pos_demand;

    void mpcOperation(float v_ref=0, float p_ref=100, float v_init=0, float p_init=0, int v_input_begin=0);
    void mpcOperation(std::vector<float> v_ref, std::vector<float> p_ref, std::vector<float> v_init, std::vector<float> p_init, std::vector<float> v_input);


    float sinePosDemand(float time);
    float cosVelDemand(float time);

    float stepPosDemand(float time);
    float stepVelDemand(float time);

    float sineToTenPosDemand(float time);
    float cosToTenVelDemand(float time);

    float powThreePosDemand(float time);
    float powTwoVelDemand(float time);

    MPC();




};





#endif