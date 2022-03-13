#ifndef PLANT_H
#define PLANT_H
#include <math.h>
#include "../unwrap.h"


#define PI 3.14159265

class PLANT{

public:
    
    bool initZPosButton = false;
    bool initXYPosButton = false;
    bool initPosButton = false;
    
    float originPos_x;
    float originPos_y;
    float originPos_z; // unwrap

    int vd_x;
    int vd_y;
    float vd_z;

    float pd_x;
    float pd_y;
    float pd_z;
    
    

    float controlInput_x_vel=0;
    float controlInput_y_vel=0;
    float controlInput_z_vel;



    
    float vel_x;
    float vel_y;
    float vel_z;

    float pos_x;
    float pos_y;
    float pos_z=0; // unwrap for pid
    float pos_z_rad=0;

    float pos_z_pre_unwrap; // for purpose of angle unwrap

    void calFspeed();

    float fspeedVel;
    float fsAngle=0;
    float fsAngle_pre = 0; // for purpose of angle unwrap
    float fsAngle_rad=0;
    float fsAngle_360=0;
    //float fsAngle_world;


    // deadReckon
    float pos_x_global=0;
    float pos_y_global=0;
    float theta_global=0;


    // position value correction
    float pos_x_pre;
    float pos_y_pre;
    //float pos_z_pre;

    float pos_x_cur;
    float pos_y_cur;
    float pos_z_cur;

    float pos_x_diff;
    float pos_y_diff;
    float pos_z_diff;

    float pos_x_correct;
    float pos_y_correct;
    float pos_z_correct; 


    void deadReckon();
    void pos_sensor_correct();
    //void pos_sensor_correct2();
    void pos_correct_to_world();
    

    

};

#endif