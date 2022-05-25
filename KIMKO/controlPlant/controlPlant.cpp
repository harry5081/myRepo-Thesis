#include "controlPlant.h"
#include <cmath>
#include <iostream>
#include <math.h>

#include <stdlib.h>     /* abs */

void PLANT::calFspeed(){

    //fspeedVel = sqrt(pow(controlInput_x_vel,2)+pow(controlInput_y_vel,2));
    //fsAngle = atan2(controlInput_y_vel,controlInput_x_vel)* 180/PI+ pos_z;
    fspeedVel = sqrt(pow(vel_x,2)+pow(vel_y,2));
    fsAngle = atan2(vel_y,vel_x)* 180/PI+ pos_z;

    //fsAngle = constrainAngle(atan2(controlInput_y_vel,controlInput_x_vel)* 180/PI+ pos_z);
    //fsAngle_rad = fsAngle* PI/180.0;
    
    //fsAngle_rad = constrainAngleRad(atan2(controlInput_y_vel,controlInput_x_vel) + pos_z_rad);
    //fsAngle = fsAngle_rad * 180.0/PI;

    // fsAngle = angleConv(atan2(controlInput_y_vel,controlInput_x_vel)* 180/PI + pos_z);
    // fsAngle_rad = fsAngle* PI/180.0;

    //fsAngle = constrainAngle(atan2(controlInput_y_vel,controlInput_x_vel)* 180/PI + pos_z);
    //fsAngle = constrainAngle(atan2(controlInput_y_vel,controlInput_x_vel)* 180/PI + pos_z);
    //float temp = (atan2(controlInput_y_vel,controlInput_x_vel)* float(180/PI))  + pos_z;
    //fsAngle = temp;

    // fsAngle = atan2(controlInput_y_vel,controlInput_x_vel)* 180/PI + pos_z;
    // fsAngle = unwrap(fsAngle_pre, fsAngle);
    // fsAngle_pre = fsAngle;
    // fsAngle_rad = fsAngle* PI/180.0;

    //fsAngle = unwrap(fsAngle_pre, fsAngle);
    //fsAngle_pre = fsAngle;

    //fsAngle_world = fsAngle + pos_z;

    // fspeed = sqrt(pow(vel_x,2)+pow(vel_y,2));
    //fsAngle = atan2(vel_y,vel_x) * 180 / PI;

    // fsAngle = atan2(controlInput_y_vel,controlInput_x_vel)* 180/PI + pos_z;
    fsAngle = unwrap(fsAngle_pre, fsAngle);
    fsAngle_360 = fmod(fsAngle,ANGLE_360);
    fsAngle_pre = fsAngle;
    
    
    fsAngle_rad = fsAngle* PI/180.0;
    // //fsAngle = angleConv(fsAngle);


    //correct to fsAngle with respect to reference
    if(fsAngle_360 - fsAngle_fromRef_360>=360){
        fsAngle_360 = fsAngle_360-360;

        fsAngle_pre = fsAngle_360;
        fsAngle_rad = fsAngle* PI/180.0;
    }

    else if(fsAngle_360 - fsAngle_fromRef_360<=-360){
        fsAngle_360 = fsAngle_360+360;
        
        fsAngle_pre = fsAngle_360;
        fsAngle_rad = fsAngle* PI/180.0;
    }
   

    
}

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

void PLANT::pos_correct_to_world(){
    pos_x_correct = cos(2.0*pos_z_rad)*pos_x + (-1.0)*sin(2.0*pos_z_rad)*pos_y;
    pos_y_correct = sin(2.0*pos_z_rad)*pos_x + cos(2.0*pos_z_rad)*pos_y;

    // pos_x_correct = cos(2.0*pos_z_raw)*pos_x + (-1.0)*sin(2.0*pos_z_raw)*pos_y;
    // pos_y_correct = sin(2.0*pos_z_raw)*pos_x + cos(2.0*pos_z_raw)*pos_y;
}

// void PLANT::pos_sensor_correct(){

//     float v_angle;
//     double angle_temp;
//     int quadrant;

//     pos_x_cur = pos_x;
//     pos_y_cur = pos_y;
//     pos_z_cur = pos_z;

//     v_angle = atan2 (vel_y,vel_x) * 180 / PI;
//     angle_temp = pos_z_cur + v_angle;

//     // std::cout <<   pos_z_cur <<std::endl;
//     // std::cout <<   v_angle <<std::endl;
//     // std::cout <<   angle_temp <<std::endl;

//     angle_temp = int(angle_temp) % 360; // [0..360) if angle is positive, (-360..0] if negative
//     if (angle_temp < 0){
//         angle_temp += 360; // Back to [0..360)
//     }
//     quadrant = (int(angle_temp/90)) % 4 + 1; // Quadrant
//     // std::cout <<   angle_temp <<std::endl;
//     // std::cout <<   quadrant <<std::endl;


//     pos_x_diff = abs(pos_x_cur-pos_x_pre);
//     pos_y_diff = abs(pos_y_cur-pos_y_pre);
//     //pos_z_diff = abs(pos_z_cur-pos_z_pre);


//     if(quadrant == 1){
//         pos_x_correct = pos_x_correct + pos_x_diff;
//         pos_y_correct = pos_y_correct + pos_y_diff;

//     }

//     else if(quadrant == 2){
//         pos_x_correct = pos_x_correct - pos_x_diff;
//         pos_y_correct = pos_y_correct + pos_y_diff;
//     }

//     else if(quadrant == 3){
//         pos_x_correct = pos_x_correct - pos_x_diff;
//         pos_y_correct = pos_y_correct - pos_y_diff;
//     }

//     else if(quadrant == 4){
//         pos_x_correct = pos_x_correct + pos_x_diff;
//         pos_y_correct = pos_y_correct - pos_y_diff;
//     }

//     else{
//         std::cout <<   "Problem!!! pos_sensor_correct()" <<std::endl;

//     }


//     pos_x_pre = pos_x_cur;
//     pos_y_pre = pos_y_cur;
//     //pos_z_pre = pos_z_cur;

    
    
    
    



// }


// void PLANT::pos_sensor_correct2(){

//     bool xdiretion = false;
//     bool ydiretion = false;

//     if( vel_x >=2 ||  vel_x <=-2){
//         xdiretion = true;
//         }
//     if(vel_y >=2 ||  vel_y <=-2){
//         ydiretion = true;
//         }

//     pos_x_cur = pos_x;
//     pos_y_cur = pos_y;
//     pos_z_cur = pos_z;

//     pos_x_diff = pos_x_cur-pos_x_pre;
//     pos_y_diff = pos_y_cur-pos_y_pre;
//     pos_z_diff = pos_z_cur-pos_z_pre;





//     if( pos_z_cur > 0.1 || pos_z_cur < -0.1){

//         if(xdiretion == true && ydiretion == false){
//             pos_x_correct = pos_x_correct + pos_x_diff;
//             pos_y_correct = pos_y_correct - pos_y_diff;
//             std::cout <<   "X" <<std::endl;

//         }

//         else if(xdiretion == false && ydiretion == true){
//             pos_x_correct = pos_x_correct - pos_x_diff;
//             pos_y_correct = pos_y_correct + pos_y_diff;
//             std::cout <<   "Y" <<std::endl;

//         }

//         else{
//             pos_x_correct = pos_x_correct + pos_x_diff;
//             pos_y_correct = pos_y_correct + pos_y_diff;
//             std::cout <<   "X & Y" <<std::endl;

//         }

//     }

//     else{

//         pos_x_correct = pos_x_correct + pos_x_diff;
//         pos_y_correct = pos_y_correct + pos_y_diff;
//         std::cout <<   "OOO" <<std::endl;
//     }

//     pos_x_pre = pos_x_cur;
//     pos_y_pre = pos_y_cur;
//     pos_z_pre = pos_z_cur;
    



// }