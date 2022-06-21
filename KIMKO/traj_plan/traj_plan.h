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
    float pre_s=0; // counter

    LEADER leaderData;

    int window = 15;

    
    std::vector<std::vector<float>> pos_ref;
    std::vector<std::vector<float>> vel_ref;

    std::vector<std::vector<float>> fspeed_ref;

    std::vector<std::vector<float>> ori_ref;
    std::vector<std::vector<float>> guess;

    void traject_from_file();
    void cir_traject_TNB();
    
    bool circleFinished = false;

    



private:

    float ds=25;

    float offset_x=0;
    float offset_y=0;

    float desireOri = 0;

    float t=0;
    float dt=0;

    float t_current;

    float r;
    float sampleTime=0.2;

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