#include "controlPlant.h"
#include <cmath>
#include <iostream>

#include <stdlib.h>     /* abs */

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

    float v_angle;
    double angle_temp;
    int quadrant;

    pos_x_cur = pos_x;
    pos_y_cur = pos_y;
    pos_z_cur = pos_z;

    v_angle = atan2 (vel_y,vel_x) * 180 / PI;
    angle_temp = pos_z_cur + v_angle;

    std::cout <<   pos_z_cur <<std::endl;
    std::cout <<   v_angle <<std::endl;
    std::cout <<   angle_temp <<std::endl;

    angle_temp = int(angle_temp) % 360; // [0..360) if angle is positive, (-360..0] if negative
    if (angle_temp < 0){
        angle_temp += 360; // Back to [0..360)
    }
    quadrant = (int(angle_temp/90)) % 4 + 1; // Quadrant
    std::cout <<   angle_temp <<std::endl;
    std::cout <<   quadrant <<std::endl;


    pos_x_diff = abs(pos_x_cur-pos_x_pre);
    pos_y_diff = abs(pos_y_cur-pos_y_pre);
    pos_z_diff = abs(pos_z_cur-pos_z_pre);


    if(quadrant == 1){
        pos_x_correct = pos_x_correct + pos_x_diff;
        pos_y_correct = pos_y_correct + pos_y_diff;

    }

    else if(quadrant == 2){
        pos_x_correct = pos_x_correct - pos_x_diff;
        pos_y_correct = pos_y_correct + pos_y_diff;
    }

    else if(quadrant == 3){
        pos_x_correct = pos_x_correct - pos_x_diff;
        pos_y_correct = pos_y_correct - pos_y_diff;
    }

    else if(quadrant == 4){
        pos_x_correct = pos_x_correct + pos_x_diff;
        pos_y_correct = pos_y_correct - pos_y_diff;
    }

    else{
        std::cout <<   "Problem!!! pos_sensor_correct()" <<std::endl;

    }


    pos_x_pre = pos_x_cur;
    pos_y_pre = pos_y_cur;
    pos_z_pre = pos_z_cur;
    



}


void PLANT::pos_sensor_correct2(){

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