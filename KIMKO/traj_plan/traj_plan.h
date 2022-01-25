#ifndef TRAJ_PLAN_H
#define TRAJ_PLAN_H

#include "../leader/leader.h"

#include <vector>
#include <math.h>
#include <time.h>

class PLANNER{

public:
    PLANNER();

    float s=0;
    float pre_s=0;

    LEADER leaderData;

    int window = 5;

    
    std::vector< std::vector<float>> pos_ref;
    std::vector< std::vector<float>> vel_ref;

    void cir_traject();




private:
    float t=0;
    float dt=0;

    float t_current;

    

    

    float r;

    

    


};

#endif