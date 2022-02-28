#ifndef TRAJ_PLAN_H
#define TRAJ_PLAN_H

#include "../leader/leader.h"
#include "../unwrap.h"

#include <vector>
#include <math.h>
#include <time.h>

#define PI 3.14159265

class PLANNER{

public:
    PLANNER();

    float s=0;
    float pre_s=0;

    LEADER leaderData;

    int window = 10;

    
    std::vector<std::vector<float>> pos_ref;
    std::vector<std::vector<float>> vel_ref;

    std::vector<std::vector<float>> fspeed_ref;

    void cir_traject();
    void cir_traject_2();

    void cir_traject_TNB();

    void linear_traject_2();
    bool circleFinished = false;





private:
    float t=0;
    float dt=0;

    float t_current;

    float r;
    float w=0.8;//0.1;
    float sampleTime=0.2;//0.2;//0.2  //0.4

    float fsAngle_pre =0;
    float fsAngle_pre_window =0;

    

    


};

#endif