#include "controlPlant.h"
#include <cmath>
#include <iostream>

void PLANT::deadReckon(){

    // float sample_time = 0.02;

    // //theta_global = pos_z;
    
    // float theta_temp = theta_global*2*PI/360;

    // pos_x_global = pos_x_global + controlInput_x_vel * cos(theta_temp) * sample_time - controlInput_y_vel* sin(theta_temp) * sample_time;
    // std::cout <<   theta_global <<std::endl;
    // std::cout <<   pos_x_global <<std::endl;
    // std::cout <<   controlInput_x_vel * cos(theta_temp) * sample_time <<std::endl;
    // std::cout <<   controlInput_y_vel* sin(theta_temp) * sample_time<<std::endl;
    
    // pos_y_global = pos_y_global + controlInput_x_vel * sin(theta_temp) * sample_time + controlInput_y_vel* cos(theta_temp) * sample_time;

    // theta_global= theta_global + controlInput_z_vel * sample_time;


    float sample_time = 0.02;

    //theta_global = pos_z;
    
    float theta_temp = theta_global*2*PI/360;

    pos_x_global = pos_x_global + vel_x * cos(theta_temp) * sample_time - vel_y* sin(theta_temp) * sample_time;
    // std::cout <<   theta_global <<std::endl;
    // std::cout <<   pos_x_global <<std::endl;
    // std::cout <<   vel_x * cos(theta_temp) * sample_time <<std::endl;
    // std::cout <<   vel_y* sin(theta_temp) * sample_time<<std::endl;
    
    pos_y_global = pos_y_global + vel_x * sin(theta_temp) * sample_time + vel_y* cos(theta_temp) * sample_time;

    theta_global= theta_global + vel_z * sample_time;

}


void PLANT::pos_sensor_correct(){

    bool xdiretion = false;
    bool ydiretion = false;

    if( vel_x >=2 ||  vel_x <=-2){
        xdiretion = true;
        }
    if(vel_y >=2 ||  vel_y <=-2){
        ydiretion = true;
        }

    pos_x_cur = pos_x;
    pos_y_cur = pos_y;
    pos_z_cur = pos_z;

    pos_x_diff = pos_x_cur-pos_x_pre;
    pos_y_diff = pos_y_cur-pos_y_pre;
    pos_z_diff = pos_z_cur-pos_z_pre;





    if( pos_z_cur > 0.1 || pos_z_cur < -0.1){

        if(xdiretion == true && ydiretion == false){
            pos_x_correct = pos_x_correct + pos_x_diff;
            pos_y_correct = pos_y_correct - pos_y_diff;
            std::cout <<   "X" <<std::endl;

        }

        else if(xdiretion == false && ydiretion == true){
            pos_x_correct = pos_x_correct - pos_x_diff;
            pos_y_correct = pos_y_correct + pos_y_diff;
            std::cout <<   "Y" <<std::endl;

        }

        else{
            pos_x_correct = pos_x_correct + pos_x_diff;
            pos_y_correct = pos_y_correct + pos_y_diff;
            std::cout <<   "X & Y" <<std::endl;

        }

    }

    else{

        pos_x_correct = pos_x_correct + pos_x_diff;
        pos_y_correct = pos_y_correct + pos_y_diff;
        std::cout <<   "OOO" <<std::endl;
    }

    pos_x_pre = pos_x_cur;
    pos_y_pre = pos_y_cur;
    pos_z_pre = pos_z_cur;
    



}