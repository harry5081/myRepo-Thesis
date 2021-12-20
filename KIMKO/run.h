#ifndef RUN_H
#define RUN_H

#include "pid.h"
#include "mpc.h"


#include "PCANBasic.h"
#include <unistd.h>
#include <stdint.h>
#include <thread>

#include "controlPlant.h"

enum DOF {X_DIRECTION=0, Y_DIRECTION, Z_DIRECTION, ALL_DIRECTION};
enum DRAW {NO_PLOT=0, PLOT};




class run
{
public:
    run();
    ~run();
    void init();
    void start();
    void canOpen();

    void initOriginPos();

    void canReadData();
    void getVelocityValue();
    void getPositionValue();

    PLANT mRobot;
    MPC mpc;

    bool velButton;
    bool posButton;

    

private:
    
        TPCANHandle m_Channel;
        TPCANBaudrate m_Btr0Btr1;
        tagTPCANMsg m_pcanMsg;
        
        
        int16_t    m_int16_desired_velocity_X;
        int16_t    m_int16_desired_velocity_Y;
        float      m_f_desired_velocity_Z;
        
        int16_t    m_int16_acceleration;
        int16_t    m_int16_deceleration;
        
        uint16_t   m_uint16_contolword;
        int16_t    m_int16_operatingmode;
        
        int16_t   m_int16_velocity_level0;
        int16_t   m_int16_velocity_level1;
        int16_t   m_int16_velocity_level2;
        
        std::shared_ptr<std::thread>                  m_Start_CanOpen;

        tagTPCANMsg m_pcanMsg_listen;
        tagTPCANMsg m_pcanMsg_writeTime;
        
};

#endif
