#ifndef PLANT_H
#define PLANT_H

#define PI 3.14159265

class PLANT{

public:
    
    bool initZPosButton = false;
    bool initXYPosButton = false;
    bool initPosButton = false;
    
    float originPos_x;
    float originPos_y;
    float originPos_z;

    int vd_x;
    int vd_y;
    float vd_z;

    float pd_x;
    float pd_y;
    float pd_z;
    
    

    float controlInput_x_vel;
    float controlInput_y_vel;
    float controlInput_z_vel;



    
    float vel_x;
    float vel_y;
    float vel_z;

    float pos_x;
    float pos_y;
    float pos_z;



    float pos_x_global=0;
    float pos_y_global=0;
    float theta_global=0;


    void deadReckon();

    

    

};

#endif