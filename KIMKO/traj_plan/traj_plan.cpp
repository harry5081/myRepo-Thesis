#include "traj_plan.h"
#include <vector>

PLANNER::PLANNER(){
    
    r = leaderData.r;

    std::vector<float> temp={0,0,0};
    
     for(int w =0;w<window;w++)
     {

        pos_ref.push_back(temp);
        vel_ref.push_back(temp);

     }


}

void PLANNER::cir_traject(){
    
    std::cout << s <<std::endl;
    dt = (s-pre_s)/r/window;

    t_current= t+dt;
    
    for(int i =0;i<window;i++){

        t=t+dt;
       

        float xt = r*sin(t);
        float yt = r*-cos(t)+r;

        std::vector<float> point = {xt,yt,0};
        pos_ref[i] = point;
        //pos_ref.push_back(point);
        //std::cout << xt    << " "<< yt <<std::endl;
               
         
        //usleep(100000);

    }

    t=t_current;

    
    pre_s = t_current*r;



}

void PLANNER::cir_traject_2(){
    
    std::cout << s <<std::endl;
    dt = sampleTime;

    t=t_current;
    

    for(int i =0;i<window;i++){

        if((t+dt)<=s/r/w){
            t=t+dt;

        }
        float xt = r*sin(w*t);
        float yt = r*-cos(w*t)+r;

        std::vector<float> point = {xt,yt,0};
        pos_ref[i] = point;
        //pos_ref.push_back(point);
        //std::cout << xt    << " "<< yt <<std::endl;
               
         
        //usleep(100000);

    }

    if((t_current+dt)<=s/r/w){
            t_current=t_current+dt;

    }

    



}