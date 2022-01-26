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

    float time;

    float s_current;
    



private:
    
    float ds = 50;

    float s=0;
    int index =0;

    
    
    void cir_traject();

    int point_cnt=7;
    float period =2;
    float startTime = 3;


    std::vector< std::vector<float>> curve;
    std::vector< std::vector<float>> vel_TN;

    



    




};

#endif