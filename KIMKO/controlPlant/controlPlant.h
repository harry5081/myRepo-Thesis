#ifndef PLANT_H
#define PLANT_H


class PLANT{

public:
    
    bool initPosButton = false;
    float originPos_z;

    int ref_vel_x;
    int ref_vel_y;
    float ref_vel_z;

    float ref_pos_x;
    float ref_pos_y;
    float ref_pos_z;
    
    

    int controlInput_x_vel;
    int controlInput_y_vel;
    float controlInput_z_vel;



    
    int vel_x;
    int vel_y;
    float vel_z;

    float pos_x;
    float pos_y;
    float pos_z;

    

    

};

#endif