#include "controlPlant.h"
#include <cmath>

void PLANT::deadReckon(){

    float sample_time = 0.02;

    theta_global = pos_z;
    
    float theta_temp = theta_global*2*PI/360;

    pos_x_global = pos_x_global + controlInput_x_vel * cos(theta_temp) * sample_time - controlInput_y_vel* sin(theta_temp) * sample_time;
    // std::cout <<   pos_x_global <<std::endl<<std::endl;
    // std::cout <<   mRobot.controlInput_x_vel * cos(theta_temp) * sample_time <<std::endl<<std::endl;
    // std::cout <<   mRobot.controlInput_y_vel* sin(theta_temp) * sample_time<<std::endl;
    
    pos_y_global = pos_y_global + controlInput_x_vel * sin(theta_temp) * sample_time + controlInput_y_vel* cos(theta_temp) * sample_time;

    theta_global= theta_global + controlInput_z_vel * sample_time;

}