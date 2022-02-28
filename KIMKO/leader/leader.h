#ifndef LEADER_H
#define LEADER_H

#include <vector>
#include <unistd.h>
#include <iostream>

class LEADER{

public:

    LEADER();

    
    float r=200;

    float xs;
    float ys;

    float tx;
    float ty;

    float nx;
    float ny;

    void cir_traject_init();
    void linear_traject_init();

    float time;

    float s_current;
    int point_cnt=64;//32;
    float period =0;

    std::vector< std::vector<float>> curve;
    



private:
    
    float ds = 100;//50*1.414;//20;

    float s=0;
    int index =0;

    
    
    void cir_traject();
    void linear_traject();
   
    
    
    float startTime = 5;


    
    std::vector< std::vector<float>> vel_TN;

    



    




};

#endif