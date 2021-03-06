#include "run.h"
#include <stdio.h>
#include <iostream>
#include <bitset>


#include <fstream>
#include <vector>
#include "matplotlib-cpp-master/matplotlibcpp.h"

#include <time.h>
#include "readWritePlot.h"


DOF dof =Z_DIRECTION;
DRAW draw = PLOT;


//PID pid_x(3,0.005,0.5,0.3);
PID pid_y(3,0.005,0.5,0.3);
PID pid_z(3,0.01,1,1);

MPC mpc;



void run::init()
{//PCAN_ERROR_OK
    
    m_Start_CanOpen.reset(new std::thread(&run::canOpen, this));
    
    m_uint16_contolword = 0x203;
    m_int16_acceleration = 200;
    m_int16_deceleration = 400;
    m_int16_operatingmode = 2;
    m_int16_velocity_level0 = 300;
    m_int16_velocity_level1 = 600;
    m_int16_velocity_level2 = 1000;
}
run::run():
m_Channel(0x41U),// PCAN_PCIBUS1 0x41U
m_Btr0Btr1(0x001CU), // PCAN_BAUD_500K
m_int16_desired_velocity_X(0),
m_int16_desired_velocity_Y(0),
m_f_desired_velocity_Z(0),
m_int16_acceleration(0),
m_int16_deceleration(0),
m_uint16_contolword(0),
m_int16_operatingmode(0),
m_int16_velocity_level0(0),
m_int16_velocity_level1(0),
m_int16_velocity_level2(0)
{
    
    while(CAN_Initialize(m_Channel, m_Btr0Btr1) != PCAN_ERROR_OK)
    {
         sleep(1);
         printf("pcan init fails \n");
    }
    
    printf("pcan init success \n");
    
    init();
    initOriginPos();



}
void run::start()
{
   
    //printf("enter the operating mode");
    //scanf("%hd", &m_int16_operatingmode);

    m_int16_operatingmode = 2;


    float time = (float)clock()/CLOCKS_PER_SEC;

    //mpc
    /////////////////////////////////////////////     X      /////////////////////////////////////
    // mRobot.ref_pos_x = mpc.sinePosDemand(time);
    // mRobot.ref_vel_x = mpc.cosVelDemand(time);

    // mRobot.ref_pos_x = mpc.sineToTenPosDemand(time);
    // mRobot.ref_vel_x = mpc.cosToTenVelDemand(time);

    /////////////////////////////////////////////     Y      /////////////////////////////////////
    //mRobot.ref_pos_y = mpc.sinePosDemand(time);
    //mRobot.ref_vel_y = mpc.cosVelDemand(time);

    /////////////////////////////////////////////     Z      /////////////////////////////////////
    mRobot.ref_pos_z = mpc.sinePosDemand(time)/5;
    mRobot.ref_vel_z = mpc.cosVelDemand(time)/5;
    
    // mRobot.ref_pos_z = mpc.sineToTenPosDemand(time)/10*3;
    // mRobot.ref_vel_z = mpc.cosToTenVelDemand(time)/10*3;
    

    //int PID::pidExe(float posError, int velDemand, float velError)
    //m_int16_desired_velocity_X = pid_x.pidExe(mRobot.ref_pos_x-mRobot.pos_x, mRobot.ref_vel_x, mRobot.ref_vel_x-mRobot.vel_x);
    //m_int16_desired_velocity_Y = pid_y.pidExe(mRobot.ref_pos_y-mRobot.pos_y, mRobot.ref_vel_y, mRobot.ref_vel_y-mRobot.vel_y);
    m_f_desired_velocity_Z = pid_z.pidExeAngle(mRobot.ref_pos_z-mRobot.pos_z,mRobot.ref_vel_z,mRobot.ref_vel_z-mRobot.vel_z);
    
    

    //ensure controller input security
    if(m_int16_desired_velocity_X >= 250){
        m_int16_desired_velocity_X=100;
        std::cout <<  "X Direction Controller Input too Fast!!!" << std::endl;
    }
    else if (m_int16_desired_velocity_X <= -250){
        m_int16_desired_velocity_X=-100;
        std::cout <<  "X Direction Controller Input too Fast!!!" << std::endl;
    }
    
    if(m_int16_desired_velocity_Y >= 250){
        m_int16_desired_velocity_Y=100;
        std::cout <<  "Y Direction Controller Input too Fast!!!" << std::endl;
    }
    else if (m_int16_desired_velocity_Y <= -250){
        m_int16_desired_velocity_Y=-100;
        std::cout <<  "Y Direction Controller Input too Fast!!!" << std::endl;
    }
    
    if(m_f_desired_velocity_Z >= 30){
        m_f_desired_velocity_Z=10;
        std::cout <<  "Z Direction Controller Input too Fast!!!" << std::endl;
    }
    else if (m_f_desired_velocity_Z <= -30){
        m_f_desired_velocity_Z=-10;
        std::cout <<  "Z Direction Controller Input too Fast!!!" << std::endl;
    }


    mRobot.controlInput_x_vel = m_int16_desired_velocity_X;
    mRobot.controlInput_y_vel = m_int16_desired_velocity_Y;
    mRobot.controlInput_z_vel = m_f_desired_velocity_Z;



    
    switch(dof){

        case X_DIRECTION:
            //printf("enter the X velocity speed :");
            //scanf("%hd", &m_int16_desired_velocity_X);
            break;

        case Y_DIRECTION:
            //printf("enter the Y velocity speed :");
            //scanf("%hd", &m_int16_desired_velocity_Y);
            break;

        case Z_DIRECTION:
            //printf("enter the Z velocity speed :");
            //scanf("%f", &m_f_desired_velocity_Z);
            //m_f_desired_velocity_Z = pid_z.pidExeAngle(robot.ref_pos_z-robot.pos_z,0,0);
            break;

        case ALL_DIRECTION:
            //printf("enter the X velocity speed :");
            //scanf("%hd", &m_int16_desired_velocity_X);

            //printf("enter the Y velocity speed :");
            //scanf("%hd", &m_int16_desired_velocity_Y);
 
            //printf("enter the Z velocity speed :");
            //scanf("%f", &m_f_desired_velocity_Z);
    
            break;
    }

    
}



run::~run()
{
}
void run::canOpen()  //CAN_Write()
{
    
    
    while(1)
    {
        uint16_t uint16_data = 0;
        uint32_t uint32_data = 0;
        //PDO 0x201
        m_pcanMsg.ID = 0x201;
        m_pcanMsg.MSGTYPE = PCAN_MODE_STANDARD;
        m_pcanMsg.LEN = 8;
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_desired_velocity_X);
        m_pcanMsg.DATA[0] = uint16_data & 0xFF;
        m_pcanMsg.DATA[1] = uint16_data>>8 & 0xFF;
        
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_desired_velocity_Y);
        m_pcanMsg.DATA[2] = uint16_data & 0xFF;
        m_pcanMsg.DATA[3] = uint16_data>>8 & 0xFF;
        
        
        uint32_data = *reinterpret_cast< uint32_t const* >(&m_f_desired_velocity_Z);
        m_pcanMsg.DATA[4] = uint32_data & 0xFF;
        m_pcanMsg.DATA[5] = uint32_data>>8 & 0xFF;
        m_pcanMsg.DATA[6] = uint32_data>>16 & 0xFF;
        m_pcanMsg.DATA[7] = uint32_data>>24 & 0xFF;       
        
        CAN_Write(m_Channel, &m_pcanMsg);

        //PDO 0x241
        m_pcanMsg.ID = 0x241;
        m_pcanMsg.MSGTYPE = PCAN_MODE_STANDARD;
        m_pcanMsg.LEN = 8;
        m_pcanMsg.DATA[0] = 0;
        m_pcanMsg.DATA[1] = 0;
        m_pcanMsg.DATA[2] = 0;
        m_pcanMsg.DATA[3] = 0;
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_acceleration);
        m_pcanMsg.DATA[4] = uint16_data & 0xFF;
        m_pcanMsg.DATA[5] = uint16_data>>8 & 0xFF;
        
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_deceleration);
        m_pcanMsg.DATA[6] = uint16_data & 0xFF;
        m_pcanMsg.DATA[7] = uint16_data>>8 & 0xFF;    
        
        CAN_Write(m_Channel, &m_pcanMsg);
      
        
        //PDO 0x261
        m_pcanMsg.ID = 0x261;
        m_pcanMsg.MSGTYPE = PCAN_MODE_STANDARD;
        m_pcanMsg.LEN = 8;
        m_pcanMsg.DATA[0] = m_uint16_contolword & 0xFF;
        m_pcanMsg.DATA[1] = m_uint16_contolword>>8 & 0xFF;
        
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_operatingmode);
        m_pcanMsg.DATA[2] = uint16_data & 0xFF;
        m_pcanMsg.DATA[3] = uint16_data>>8 & 0xFF;    
        
        m_pcanMsg.DATA[4] = 0;
        m_pcanMsg.DATA[5] = 0;
        m_pcanMsg.DATA[6] = 0;
        m_pcanMsg.DATA[7] = 0;
        
        CAN_Write(m_Channel, &m_pcanMsg);
        
        
        //PDO 0x321
        m_pcanMsg.ID = 0x321;
        m_pcanMsg.MSGTYPE = PCAN_MODE_STANDARD;
        m_pcanMsg.LEN = 8;
        
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_velocity_level0);
        m_pcanMsg.DATA[0] = uint16_data & 0xFF;
        m_pcanMsg.DATA[1] = uint16_data>>8 & 0xFF;   
        
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_velocity_level1);
        m_pcanMsg.DATA[2] = uint16_data & 0xFF;
        m_pcanMsg.DATA[3] = uint16_data>>8 & 0xFF;    
        
        uint16_data = *reinterpret_cast< uint16_t const* >(&m_int16_velocity_level2);
        m_pcanMsg.DATA[4] = uint16_data & 0xFF;
        m_pcanMsg.DATA[5] = uint16_data>>8 & 0xFF;   
        
        CAN_Write(m_Channel, &m_pcanMsg);
        
        
        //Heartbeat
        m_pcanMsg.ID = 0x702;
        m_pcanMsg.MSGTYPE = PCAN_MODE_STANDARD;
        m_pcanMsg.LEN = 8;
        static uint8_t uint8_step = 0;
        switch(uint8_step)
        {
            case 0:
            {
                m_pcanMsg.DATA[0] = 5;
                uint8_step = 1;
                break;
            }
            case 1:
            {
                m_pcanMsg.DATA[0] = 10;
                uint8_step = 0;
                break;
            }
        }
        
        m_pcanMsg.DATA[1] = 0;
        m_pcanMsg.DATA[2] = 0;
        m_pcanMsg.DATA[3] = 0;
        m_pcanMsg.DATA[4] = 0;
        m_pcanMsg.DATA[5] = 0;
        m_pcanMsg.DATA[6] = 0;
        m_pcanMsg.DATA[7] = 0;
        
        CAN_Write(m_Channel, &m_pcanMsg);
        
        usleep(20000);
        
    }
}

void run::canReadData(){

    TPCANStatus result;
       
   while(1){
 
        result = CAN_Read(m_Channel, &m_pcanMsg_listen, nullptr);
                
        if(result != PCAN_ERROR_QRCVEMPTY){
            
            // data processing

            getVelocityValue();
            getPositionValue();
          
         }
    
    }
    
}

void run::getVelocityValue(){

    if(m_pcanMsg_listen.ID == 0x181){

        int16_t x_vel = static_cast<uint16_t>(m_pcanMsg_listen.DATA[0]) | static_cast<uint16_t>(m_pcanMsg_listen.DATA[1]<<8);
        //std::cout << "x_vel: ";
        //std::cout << std::dec << x_vel << std::endl;
 
 
        int16_t y_vel = static_cast<uint16_t>(m_pcanMsg_listen.DATA[2]) | static_cast<uint16_t>(m_pcanMsg_listen.DATA[3]<<8);
        //std::cout << "y_vel: ";
        //std::cout << std::dec << y_vel << std::endl;


        uint32_t z_temp=static_cast<uint32_t>(m_pcanMsg_listen.DATA[4])|static_cast<uint32_t>(m_pcanMsg_listen.DATA[5]<<8)\
                            |static_cast<uint32_t>(m_pcanMsg_listen.DATA[6]<<16)|static_cast<uint32_t>(m_pcanMsg_listen.DATA[7]<<24);
        float z_vel = *((float *) &z_temp);
        //std::cout << "z_vel: ";
        //std::cout <<  z_vel << std::endl;

        mRobot.vel_x = x_vel;
        mRobot.vel_y = y_vel;
        mRobot.vel_z = z_vel;


        if(draw == PLOT){
            // canReadTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/CAN_Read_Time");
            
            // canReadDataPlot_X->writeDatatoFile(mRobot.vel_x, "plot/CAN_Read_Data_X");
            // canReadDataPlot_Y->writeDatatoFile(mRobot.vel_y, "plot/CAN_Read_Data_Y");
            // canReadDataPlot_Z->writeDatatoFile(mRobot.vel_z, "plot/CAN_Read_Data_Z");
            
        }
    }

}

void run::getPositionValue(){
    
    //X,Y Position  
    if(m_pcanMsg_listen.ID == 0x1A1){
                

        int32_t x_pos = static_cast<uint32_t>(m_pcanMsg_listen.DATA[0]) | static_cast<uint32_t>(m_pcanMsg_listen.DATA[1]<<8)\
                            |static_cast<uint32_t>(m_pcanMsg_listen.DATA[2])<<16 | static_cast<uint32_t>(m_pcanMsg_listen.DATA[3]<<24);
        //std::cout << "x_pos: ";
        //std::cout << std::dec << x_pos << std::endl;

        int32_t y_pos = static_cast<uint32_t>(m_pcanMsg_listen.DATA[4]) | static_cast<uint32_t>(m_pcanMsg_listen.DATA[5]<<8)\
                            |static_cast<uint32_t>(m_pcanMsg_listen.DATA[6])<<16 | static_cast<uint32_t>(m_pcanMsg_listen.DATA[7]<<24);
        //    std::cout << "y_pos: ";
        //    std::cout << std::dec << y_pos << std::endl;

        // mRobot.pos_x = x_pos;
        // mRobot.pos_y = y_pos;   

        mRobot.pos_x = x_pos-mRobot.originPos_x;
        mRobot.pos_y = y_pos-mRobot.originPos_y; 

    } //if(m_pcanMsg_listen.ID == 0x1A1)


    //Z Position
    if(m_pcanMsg_listen.ID == 0x1C1){
                
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[0]) << std::endl;
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[1]) << std::endl;
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[2]) << std::endl;
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[3]) << std::endl;
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[4]) << std::endl;
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[5]) << std::endl;
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[6]) << std::endl;
        // std::cout << std::dec << static_cast<uint32_t>(m_pcanMsg_listen.DATA[7]) << std::endl;
        // std::cout << "---------------------------------------------------------" << std::endl;
                
        uint32_t z_pos_temp=static_cast<uint32_t>(m_pcanMsg_listen.DATA[0])|static_cast<uint32_t>(m_pcanMsg_listen.DATA[1]<<8)\
                        |static_cast<uint32_t>(m_pcanMsg_listen.DATA[2]<<16)|static_cast<uint32_t>(m_pcanMsg_listen.DATA[3]<<24);
        float z_pos = *((float *) &z_pos_temp);
        // std::cout << "z_pos: ";
        // std::cout <<  z_pos << std::endl;

        mRobot.pos_z = z_pos-mRobot.originPos_z;
       
        //mRobot.pos_z = z_pos;

         if(mRobot.pos_z>=180 && mRobot.pos_z<=360){
             mRobot.pos_z=mRobot.pos_z-360;
             std::cout <<  "Angle 180~360 convert to -180~0" << std::endl;
             std::cout <<  "Angle 180~360 convert to -180~0" << std::endl;
             std::cout <<  "Angle 180~360 convert to -180~0" << std::endl;
        }

        else if(mRobot.pos_z>=-360 && mRobot.pos_z<=-180){
             mRobot.pos_z=mRobot.pos_z+360;
             std::cout <<  "Angle -180~-360 convert to 180~0" << std::endl;
             std::cout <<  "Angle -180~-360 convert to 180~0" << std::endl;
             std::cout <<  "Angle -180~-360 convert to 180~0" << std::endl;
        }
        
    }// if(m_pcanMsg_listen.ID == 0x1C1)
            



}


void run::initOriginPos(){
              //X,Y Position

    while(mRobot.initPosButton == false)   {
    
        //std::cout <<  "initOriginPos" << std::endl;
        TPCANStatus result;
        result = CAN_Read(m_Channel, &m_pcanMsg_listen, nullptr);
    
        //XY Position            
        if(m_pcanMsg_listen.ID == 0x1A1){
            int32_t x_pos = static_cast<uint32_t>(m_pcanMsg_listen.DATA[0]) | static_cast<uint32_t>(m_pcanMsg_listen.DATA[1]<<8)\
                       |static_cast<uint32_t>(m_pcanMsg_listen.DATA[2])<<16 | static_cast<uint32_t>(m_pcanMsg_listen.DATA[3]<<24);
            mRobot.originPos_x = x_pos;

            int32_t y_pos = static_cast<uint32_t>(m_pcanMsg_listen.DATA[4]) | static_cast<uint32_t>(m_pcanMsg_listen.DATA[5]<<8)\
                       |static_cast<uint32_t>(m_pcanMsg_listen.DATA[6])<<16 | static_cast<uint32_t>(m_pcanMsg_listen.DATA[7]<<24);
            mRobot.originPos_y = y_pos;
            
            mRobot.initXYPosButton=true;

            

       }

        //Z Position

        if(m_pcanMsg_listen.ID == 0x1C1){
               
            uint32_t z_pos_temp=static_cast<uint32_t>(m_pcanMsg_listen.DATA[0])|static_cast<uint32_t>(m_pcanMsg_listen.DATA[1]<<8)\
                                |static_cast<uint32_t>(m_pcanMsg_listen.DATA[2]<<16)|static_cast<uint32_t>(m_pcanMsg_listen.DATA[3]<<24);
            float z_pos = *((float *) &z_pos_temp);
        
            mRobot.originPos_z = z_pos;
            mRobot.initZPosButton=true;   

            

        if(mRobot.initXYPosButton==true && mRobot.initZPosButton==true){
            mRobot.initPosButton =true;
            std::cout <<  std::endl;
            std::cout <<  std::endl;
            std::cout <<  "initOrigin X Pos: " << mRobot.originPos_x <<std::endl;
            std::cout <<  "initOrigin Y Pos: " << mRobot.originPos_y <<std::endl;
            std::cout <<  "initOrigin Z Pos: " << mRobot.originPos_z << std::endl;   
            std::cout <<  std::endl;
        }

        }

    }         
    
 
}

