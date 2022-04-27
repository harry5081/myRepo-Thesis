#ifndef TRAJ_PLAN_H
#define TRAJ_PLAN_H

#include "../leader/leader.h"
#include "../unwrap.h"

#include <vector>
#include <math.h>
#include <time.h>
#include <iostream>
#include <string>
#include <fstream>

#define PI 3.14159265

class PLANNER{

public:
    PLANNER();

    float s=0;
    float s_pre_route=700;
    float pre_s=0; // counter

    LEADER leaderData;

    int window = 2;

    
    std::vector<std::vector<float>> pos_ref;
    std::vector<std::vector<float>> vel_ref;

    std::vector<std::vector<float>> fspeed_ref;

    std::vector<std::vector<float>> ori_ref;
    std::vector<std::vector<float>> guess;

    void cir_traject();
    void cir_traject_2();

    void cir_traject_TNB();
    void cir_traject_TNB_preAngle();

    void linear_traject_2();
    bool circleFinished = false;

    void traject_from_file();



private:

    float ds=20;

    float offset_x=0;
    float offset_y=0;

    float desireOri = 0;

    float t=0;
    float dt=0;

    float t_current;

    float r;
    float w=0.8;//0.1;
    float sampleTime=0.2;//0.2;//0.2  //0.4

    float fsAngle_pre =0;
    float fsAngle_pre_window =0;
    float fsAngle_2PI =0;

    
    
    void readTrajFile();
    std::vector<int>index_f;
    std::vector<float>x_Pos_f;
	std::vector<float>y_Pos_f;
    std::vector<float>phi_ref_f;
	std::vector<float>s_Dot_f;
    std::vector<float>blank_ref_f;
	std::vector<float>w_ref_f;
    std::vector<float>ori_ref_f;
    int lineCount=0;
    


};

#endif