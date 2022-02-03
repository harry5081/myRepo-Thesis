#include "leader.h"
#include <math.h>

LEADER::LEADER(){

    std::cout <<  "LEADER init"<<std::endl;
    

}

void LEADER::cir_traject_init(){
    for(int i=0;i<point_cnt;i++){

        float xs_temp = r*sin(s/r);
        float ys_temp = r*-cos(s/r)+r;

        std::vector<float> point = {xs_temp,ys_temp};
        s=s+ds;

        curve.push_back(point);
        
    }

    float pre_time=0;

    while(1){

        usleep(100000);  
        float time = (float)clock()/CLOCKS_PER_SEC;
        
        if(time>=startTime){

            if(time-pre_time>=period){
            
                cir_traject();
                pre_time = time;

            }

        }//if(time>=startTime)

        
        
    }//while(1){
    

    
}

void LEADER::cir_traject(){

    //index = int((time-startTime)/period);
    
    
    if(index<point_cnt){
        s_current = index*ds;
        xs =curve[index][0];
        ys =curve[index][1];
        index++;
        // std::cout << "Leader: "<< s_current <<std::endl;
        // std::cout << "Leader: "<< xs<<  " "<<  ys<<std::endl;
        // std::cout << time <<std::endl;
        
    }
    


}

void LEADER::linear_traject_init(){
    for(int i=0;i<point_cnt;i++){

        float xs_temp = s;
        float ys_temp = 0;

        std::vector<float> point = {xs_temp,ys_temp};
        s=s+ds;

        curve.push_back(point);
        
    }

    float pre_time=0;

    while(1){
        usleep(100000);  
        float time = (float)clock()/CLOCKS_PER_SEC;
        
        if(time>=startTime){

            if(time-pre_time>=period){
            
                linear_traject();
                pre_time = time;

            }

        }//if(time>=startTime)

        
        
    }//while(1){
    

    
}

void LEADER::linear_traject(){

    //index = int((time-startTime)/period);
    
    
    if(index<point_cnt){
        s_current = index*ds;
        xs =curve[index][0];
        ys =curve[index][1];
        index++;
        std::cout << "Leader: "<< s_current <<std::endl;
        std::cout << "Leader: "<< xs<<  " "<<  ys<<std::endl;
        // std::cout << time <<std::endl;
        
    }
    


}



