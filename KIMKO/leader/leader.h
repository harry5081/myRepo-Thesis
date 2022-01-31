#ifndef LEADER_H
#define LEADER_H

#include <vector>
#include <unistd.h>
#include <iostream>

class LEADER{

public:

    LEADER();

    
    float r=100;

    float xs;
    float ys;

    float tx;
    float ty;

    float nx;
    float ny;

    void cir_traject_init();

    float time;

    float s_current;
    int point_cnt=32;//32;
    float period =0;

    std::vector< std::vector<float>> curve;
    



private:
    
    float ds = 20;

    float s=0;
    int index =0;

    
    
    void cir_traject();
   
    
    
    float startTime = 10;


    
    std::vector< std::vector<float>> vel_TN;

    



    




};

#endif