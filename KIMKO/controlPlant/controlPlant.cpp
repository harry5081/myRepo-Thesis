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

    fsAngle = unwrap(fsAngle_pre, fsAngle); // make it continuous
    fsAngle_360 = fmod(fsAngle,ANGLE_360);  // constraint to -360~360
    fsAngle_pre = fsAngle;
        
    fsAngle_rad = fsAngle* PI/180.0;
    

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



