#ifndef LEADER_H
#define LEADER_H

#include <vector>
#include <unistd.h>
#include <iostream>
#include <iostream>
#include <string>
#include <fstream>


class LEADER{

public:

    LEADER();

    // user input
    int point_cnt=26; //64;//32; // 26 for r=400

    
    float r=400;//500000;//200;

    float xs;
    float ys;

    float tx;
    float ty;

    float nx;
    float ny;

    void cir_traject_init();
    void file_traject_init();
    //void linear_traject_init();

    float time;

    float s_current;
    
    
    float period =0;

    std::vector< std::vector<float>> curve;
    
    void traject_from_file();
    void readTrajFile();


private:

    float offset_xs=0;
    float offset_ys=0;


    // user input
    float startTime = 0;
    float ds = 100;//50*1.414;//20;


    float s=0;
    int index =0;

    
    
    void cir_traject();
    //void linear_traject();
   
    
    
    


    
    std::vector< std::vector<float>> vel_TN;


    
    // std::vector<int>index_f;
    // std::vector<float>x_Pos_f;
	// std::vector<float>y_Pos_f;
    // std::vector<float>phi_ref_f;
	// std::vector<float>s_Dot_f;
    // std::vector<float>blank_ref_f;
	// std::vector<float>w_ref_f;
    int lineCount=0;

    



    




};

#endif