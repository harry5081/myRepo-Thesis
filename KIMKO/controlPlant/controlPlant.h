#ifndef PLANT_H
#define PLANT_H


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