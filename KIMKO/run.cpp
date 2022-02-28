#include "run.h"
#include <stdio.h>
#include <iostream>
#include <bitset>


#include <fstream>
#include <vector>
#include "matplotlib-cpp-master/matplotlibcpp.h"


#include <time.h>
#include "readWritePlot.h"

 #include <pybind11/pybind11.h>
 #include <pybind11/embed.h>
 #include <pybind11/stl.h>
//#include <pybind11/numpy.h>

#define PI 3.14159265

 float timeDiff;
 float timeTemp;


DOF dof =X_DIRECTION;
DRAW draw = PLOT;

//PID pid_x(3,0,0,0);
PID pid_x(3,0,1,0.3);
PID pid_y(3,0,1,0.3);

//PID pid_z(3,0.01,1,1);
//PID pid_z(3,0,1,1);
PID pid_z(2,0,1,0.1);



// bool velButton;
// bool posButton;



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
        printf("pcan init fails.. \n");
        
        
        

        //mpc.mpcOperation(0,100,0,0,0);
        
    
    }

    
   
    printf("pcan init success.. \n");
    
    init();
    initOriginPos();



}
void run::start()
{
    
   
    

    //CAN_READ to get the initial state
   
    //printf("enter the operating mode");
    //scanf("%hd", &m_int16_operatingmode);

    m_int16_operatingmode = 2;


    float time = (float)clock()/CLOCKS_PER_SEC;
    timeDiff=time - timeTemp;
    std::cout << "!!!!!!!!!!!!!!!!!!!  TimeTest  !!!!!!!!!!!!!!!!!!!!!    "<< timeDiff <<std::endl;
    timeTemp=time;
    
    //usleep(150000);
    usleep(150000);
    
    //planner.linear_traject_2();
    //planner.cir_traject_2();
    planner.cir_traject_TNB();
    
    // std::vector<std::vector<float>> v_ref_dyn = planner.vel_ref;
    // std::vector<std::vector<float>> p_ref_dyn = planner.pos_ref;
    // std::vector<std::vector<float>> fspeed_ref = planner.fspeed_ref;

    // mpc.x_vel_ref = planner.vel_ref[0][0]; //plot
    // mpc.y_vel_ref = planner.vel_ref[0][1]; //plot

    // mpc.x_pos_ref = planner.pos_ref[0][0]; //plot
    // mpc.y_pos_ref = planner.pos_ref[0][1]; //plot

    // mpc.z_pos_ref = planner.pos_ref[0][2]; //plot

    // mpc.fspeedVel_ref = planner.fspeed_ref[0][0]; //plot
    // mpc.fsAngle_ref = planner.fspeed_ref[0][1]; //plot


    
    // std::vector<float> v_ref = {mpc.x_vel_ref, mpc.y_vel_ref, mpc.z_vel_ref};
    // std::vector<float> p_ref = {mpc.x_pos_ref, mpc.y_pos_ref, mpc.z_pos_ref};
    
    
    // std::vector<float> v_init = {mRobot.vel_x, mRobot.vel_y, mRobot.vel_z};
    // std::vector<float> p_init = {mRobot.pos_x_correct, mRobot.pos_y_correct, mRobot.pos_z};
    // std::vector<float> v_input = {mRobot.controlInput_x_vel, mRobot.controlInput_y_vel, mRobot.controlInput_z_vel};
    // std::vector<float> fspeed_init = {mRobot.fspeedVel, mRobot.fsAngle};
    

    float time1 = (float)clock()/CLOCKS_PER_SEC;
    //mpc vd pd from python
    
    //std::cout<<std::endl;
    //std::cout << "Robot Frame Value"<<std::endl;
    //std::cout <<  "vel_x: " << mRobot.vel_x <<  "     pos_x: " << mRobot.pos_x <<std::endl<<std::endl;   
    
    //mpc.mpcOperation(mpc.x_vel_ref, mpc.x_pos_ref, mRobot.vel_x, mRobot.pos_x_correct, mRobot.controlInput_x_vel);  // one direction
    //mpc.mpcOperation(v_ref, p_ref, v_init, p_init, v_input); // three dimention with fix ref
    //mpc.mpcOperation(v_ref_dyn, p_ref_dyn, v_init, p_init, v_input); // dyn ref
    //mpc.mpcOperation(v_ref_dyn, p_ref_dyn, v_init, p_init, v_input, fspeed_ref, fspeed_init); // dyn ref fspeed





    ///////////////////////////////////  Error Dynamic  ///////////////////////////////////
    //fix Ref for x

    // std::vector<float> p_ref = {mpc.x_pos_ref, mpc.y_pos_ref, 0};
    // std::vector<float> v_ref = {mpc.x_vel_ref,0 , 0};
    
    // std::vector<float> p_init = {mRobot.pos_x_correct, mRobot.pos_y_correct, 0};
    // std::vector<float> v_init = {mRobot.vel_x, 0, 0};
    
    // mpc.mpcErrDyn(p_ref, v_ref, p_init, v_init);

    //fix Ref for y
    // std::vector<float> p_ref = {mpc.x_pos_ref, mpc.y_pos_ref, mpc.fsAngle_ref*float(PI/180)};
    // std::vector<float> v_ref = {mpc.fspeedVel_ref, 0, 0};
    
    // std::vector<float> p_init = {mRobot.pos_x_correct, mRobot.pos_y_correct, mRobot.fsAngle*float(PI/180)};
    // std::vector<float> v_init = {mRobot.vel_y, 0, 0};
    
    // mpc.mpcErrDyn(p_ref, v_ref, p_init, v_init);


    //dyn Ref
    // for x
    // std::vector<std::vector<float>> v_ref_dyn = planner.vel_ref;
    // std::vector<std::vector<float>> p_ref_dyn = planner.pos_ref;
    // //std::vector<std::vector<float>> fspeed_ref = planner.fspeed_ref;

    // std::vector<float> p_init = {mRobot.pos_x_correct, mRobot.pos_y_correct, 0};
    // std::vector<float> v_init = {mRobot.vel_x, 0, 0};

    // mpc.x_vel_ref = planner.vel_ref[0][0]; //plot
    // mpc.y_vel_ref = planner.vel_ref[0][1]; //plot

    // mpc.x_pos_ref = planner.pos_ref[0][0]; //plot
    // mpc.y_pos_ref = planner.pos_ref[0][1]; //plot

    // mpc.z_pos_ref = planner.pos_ref[0][2]; //plot

    // mpc.mpcErrDyn(p_ref_dyn, v_ref_dyn, p_init, v_init);

    //dyn Ref
    // for xy

    std::vector<std::vector<float>> v_ref_dyn = planner.vel_ref;
    std::vector<std::vector<float>> p_ref_dyn = planner.pos_ref;
    //std::vector<std::vector<float>> fspeed_ref = planner.fspeed_ref;

    float fspeed_temp = sqrt(pow(mRobot.vel_x,2) + pow(mRobot.vel_y,2));

    std::vector<float> p_init = {mRobot.pos_x, mRobot.pos_y, mRobot.fsAngle*float(PI/180)};
    //std::vector<float> p_init = {mRobot.pos_x, mRobot.pos_y, mRobot.fsAngle*float(PI/180)};
    //std::vector<float> p_init = {mRobot.pos_x, mRobot.pos_y, planner.fsAngle};
    std::vector<float> v_init = {fspeed_temp, 0, 0};
    //std::vector<float> v_init = {fspeed_temp, 0, 0};
    //std::vector<float> v_init = {mRobot.fspeedVel, 0, 0};
    //std::vector<float> p_init = {mRobot.pos_x_correct, mRobot.pos_y_correct, mRobot.fsAngle*float(PI/180)};
     

    //mpc.x_vel_ref = planner.vel_ref[0][0]; //plot
    //mpc.y_vel_ref = planner.vel_ref[0][1]; //plot

    mpc.x_pos_ref = planner.pos_ref[0][0]; //plot
    mpc.y_pos_ref = planner.pos_ref[0][1]; //plot
    mpc.fsAngle_ref = planner.pos_ref[0][2]*180/PI; //plot
    mpc.fspeedVel_ref = planner.vel_ref[0][0]; //plot


    mpc.mpcErrDyn_xy(p_ref_dyn, v_ref_dyn, p_init, v_init);

    // mpc.x_vel_demand=mpc.fspeedVel_demand*cos((mpc.fsAngle_demand)*PI/180);
    // mpc.y_vel_demand=mpc.fspeedVel_demand*sin((mpc.fsAngle_demand)*PI/180);

    mpc.x_vel_demand=mpc.fspeedVel_demand*cos((mpc.fsAngle_demand-mRobot.pos_z)*PI/180);
    mpc.y_vel_demand=mpc.fspeedVel_demand*sin((mpc.fsAngle_demand-mRobot.pos_z)*PI/180);
    
    

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    //std::cout << " MPC_XYZ operation time: "<< time2-time1 << std::endl;


    /////////////////////////////////////////////     X      /////////////////////////////////////
    mRobot.vd_x = mpc.x_vel_demand;
    mRobot.pd_x = mpc.x_pos_demand;
    
    
    //mRobot.pd_x = mpc.sinePosDemand(time);
    //mRobot.vd_x = mpc.cosVelDemand(time);

    


    //mRobot.pd_x = mpc.stepPosDemand(time);
    //mRobot.vd_x = mpc.stepVelDemand(time);

    /////////////////////////////////////////////     Y      /////////////////////////////////////
    //std::cout<<std::endl;
    //std::cout << "Robot Value"<<std::endl;
    //std::cout <<  "vel_y: " << mRobot.vel_y <<  "     pos_y: " << mRobot.pos_y <<std::endl<<std::endl;   
    
    mRobot.vd_y = mpc.y_vel_demand;
    mRobot.pd_y = mpc.y_pos_demand;
    
    //mRobot.pd_y = mpc.sinePosDemand(time);
    //mRobot.vd_y = mpc.cosVelDemand(time);

    /////////////////////////////////////////////     Z      /////////////////////////////////////
    //std::cout <<  "vel_z: " << mRobot.vel_z <<  "     pos_z: " << mRobot.pos_z <<std::endl<<std::endl;   
    //mRobot.vd_z = mpc.z_vel_demand;
    //mRobot.pd_z = mpc.z_pos_demand;
    
    mRobot.pd_z = 3*mpc.sinePosDemand(time)/10;
    mRobot.vd_z = 3*mpc.cosVelDemand(time)/10;
    
    // mRobot.pd_z = -mpc.sineToTenPosDemand(time)/10;
    // mRobot.vd_z = -mpc.cosToTenVelDemand(time)/10;

    // mRobot.pd_x = 0;
    // mRobot.vd_x = 0;

    // mRobot.pd_x = mpc.sineToTenPosDemand(time);
    // mRobot.vd_x = mpc.cosToTenVelDemand(time);

    // mRobot.pd_y = 0;
    // mRobot.vd_y = 0;

    // mRobot.vd_z =0;
    // mRobot.pd_z = 0;

    //mRobot.pd_y = mpc.sineToTenPosDemand(time);
    //mRobot.vd_y = mpc.cosToTenVelDemand(time);

    // mRobot.pd_x = (1)*mpc.sineToTenPosDemand(time);
    // mRobot.vd_x = (1)*mpc.cosToTenVelDemand(time);
    
    // mRobot.pd_z = 1*mpc.sineToTenPosDemand(time)/10;
    // mRobot.vd_z = 1*mpc.cosToTenVelDemand(time)/10;

    // mRobot.pd_z = (-1)*mpc.sineToTenPosDemand(time)/10;
    // mRobot.vd_z = (-1)*mpc.cosToTenVelDemand(time)/10;
    
    

    //int PID::pidExe(float posError, int velDemand, float velError)
    // m_int16_desired_velocity_X = pid_x.pidExe(mRobot.pd_x-mRobot.pos_x_correct, mRobot.vd_x, mRobot.vd_x-mRobot.vel_x);
    // m_int16_desired_velocity_Y = pid_y.pidExe(mRobot.pd_y-mRobot.pos_y_correct, mRobot.vd_y, mRobot.vd_y-mRobot.vel_y);
    // m_f_desired_velocity_Z = pid_z.pidExeAngle(mRobot.pd_z-mRobot.pos_z,mRobot.vd_z,mRobot.vd_z-mRobot.vel_z);

    m_int16_desired_velocity_X = pid_x.pidExe(mRobot.pd_x-mRobot.pos_x, mRobot.vd_x, mRobot.vd_x-mRobot.vel_x);
    m_int16_desired_velocity_Y = pid_y.pidExe(mRobot.pd_y-mRobot.pos_y, mRobot.vd_y, mRobot.vd_y-mRobot.vel_y);
    m_f_desired_velocity_Z = pid_z.pidExeAngle(mRobot.pd_z-mRobot.pos_z,mRobot.vd_z,mRobot.vd_z-mRobot.vel_z);
    

    // switch(dof){

    //     case X_DIRECTION:
    //         printf("enter the X velocity speed :");
    //         scanf("%hd", &m_int16_desired_velocity_X);
    //         break;

    //     case Y_DIRECTION:
    //         //printf("enter the Y velocity speed :");
    //         //scanf("%hd", &m_int16_desired_velocity_Y);
    //         break;

    //     case Z_DIRECTION:
    //         printf("enter the Z velocity speed :");
    //         scanf("%f", &m_f_desired_velocity_Z);
            
    //         break;

    //     case ALL_DIRECTION:
    //         printf("enter the X velocity speed :");
    //         scanf("%hd", &m_int16_desired_velocity_X);

    //         printf("enter the Y velocity speed :");
    //         scanf("%hd", &m_int16_desired_velocity_Y);
 
    //         printf("enter the Z velocity speed :");
    //         scanf("%f", &m_f_desired_velocity_Z);
    
    //         break;
    // }
    

    //ensure controller input security
    if(m_int16_desired_velocity_X > 320){
        m_int16_desired_velocity_X=100;
        std::cout <<  "X Direction Controller Input too Fast!!!" << std::endl;
    }
    else if (m_int16_desired_velocity_X < -320){
        m_int16_desired_velocity_X=-100;
        std::cout <<  "X Direction Controller Input too Fast!!!" << std::endl;
    }
    
    if(m_int16_desired_velocity_Y > 280){
        m_int16_desired_velocity_Y=100;
        std::cout <<  "Y Direction Controller Input too Fast!!!" << std::endl;
    }
    else if (m_int16_desired_velocity_Y < -280){
        m_int16_desired_velocity_Y=-100;
        std::cout <<  "Y Direction Controller Input too Fast!!!" << std::endl;
    }
    
    if(m_f_desired_velocity_Z >= 15){
        m_f_desired_velocity_Z=5;
        std::cout <<  "Z Direction Controller Input too Fast!!!" << std::endl;
    }
    else if (m_f_desired_velocity_Z <= -15){
        m_f_desired_velocity_Z=-5;
        std::cout <<  "Z Direction Controller Input too Fast!!!" << std::endl;
    }

    //m_int16_desired_velocity_X = 10;
    //m_int16_desired_velocity_Y = 0;
    //m_f_desired_velocity_Z = 2;

    mRobot.controlInput_x_vel = m_int16_desired_velocity_X;
    mRobot.controlInput_y_vel = m_int16_desired_velocity_Y;
    mRobot.controlInput_z_vel = m_f_desired_velocity_Z;

    // std::cout <<  "V_input_x: " << m_int16_desired_velocity_X<< std::endl;
    // std::cout <<  "V_input_y: " << m_int16_desired_velocity_Y<< std::endl;
    // std::cout <<  "V_input_z: " << m_f_desired_velocity_Z<< std::endl;

    

    // wait till pos and vel both are read from CAN_READ 
    // while(velButton ==false ||  posButton==false){
   
        
    // }

    // velButton =false;
    // posButton =false;


    //  while(velButton ==false ||  posButton==false){
    //     canReadData();
    // }

    // velButton =false;
    // posButton =false;
    

    
}



run::~run()
{
}
void run::canOpen()  //CAN_Write()
{
    
    
    while(1)
    {
        //std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
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

   
        
        usleep(20000);  //20 ms

        
        
        
    }
}

void run::canReadData(){

    TPCANStatus result;
       
   while(1){
 
        result = CAN_Read(m_Channel, &m_pcanMsg_listen, nullptr);
                
        if(result != PCAN_ERROR_QRCVEMPTY){
            //std::cout << "---------------------------------" << std::endl;
            // data processing

            getVelocityValue();
            getPositionValue();

            mRobot.pos_sensor_correct();
            mRobot.calFspeed();

            // // data processing
            // std::cout << "Robot Frame Value"<<std::endl;
            // std::cout <<  "     pos_x: " << mRobot.pos_x <<std::endl<<std::endl;
            // std::cout <<  "     pos_y: " << mRobot.pos_y <<std::endl<<std::endl;
            // std::cout <<  "     pos_z: " << mRobot.pos_z <<std::endl<<std::endl;

            // mRobot.deadReckon();
            // std::cout << "Global Frame Value"<<std::endl;
            // std::cout <<  "     World_x: " << mRobot.pos_x_global <<std::endl<<std::endl;
            // std::cout <<  "     World_y: " << mRobot.pos_y_global <<std::endl<<std::endl;
            // std::cout <<  "     World_theta: " << mRobot.theta_global <<std::endl<<std::endl;

            
            // std::cout << "Correct Position Value"<<std::endl;
            // std::cout <<  "     Correct_x: " << mRobot.pos_x_correct <<std::endl<<std::endl;
            // std::cout <<  "     Correct_y: " << mRobot.pos_y_correct <<std::endl<<std::endl;
            // std::cout <<  "     Correct_theta: " << mRobot.pos_z_correct <<std::endl<<std::endl;
            
            usleep(1000); //1 ms
         }

        //usleep(20000);
    
    }
    
}

void run::getVelocityValue(){

    if(m_pcanMsg_listen.ID == 0x181){
        velButton=true;
        //std::cout << "-----VEL-----VEL-----VEL-----VEL-----VEL-----VEL-----" << std::endl;
            

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
        posButton=true;
        //std::cout << "-----POS-----POS-----POS-----POS-----POS-----POS-----" << std::endl;        

        int32_t x_pos = static_cast<uint32_t>(m_pcanMsg_listen.DATA[0]) | static_cast<uint32_t>(m_pcanMsg_listen.DATA[1]<<8)\
                            |static_cast<uint32_t>(m_pcanMsg_listen.DATA[2])<<16 | static_cast<uint32_t>(m_pcanMsg_listen.DATA[3]<<24);
        //std::cout << "x_pos: ";
        //std::cout << std::dec << x_pos << std::endl;

        int32_t y_pos = static_cast<uint32_t>(m_pcanMsg_listen.DATA[4]) | static_cast<uint32_t>(m_pcanMsg_listen.DATA[5]<<8)\
                            |static_cast<uint32_t>(m_pcanMsg_listen.DATA[6])<<16 | static_cast<uint32_t>(m_pcanMsg_listen.DATA[7]<<24);
        //    std::cout << "y_pos: ";
        //std::cout << "-----POS-----POS-----POS-----POS-----POS-----POS-----" << std::endl;   
        //std::cout << std::dec << y_pos << std::endl;

        // mRobot.pos_x = x_pos;
        // mRobot.pos_y = y_pos;   

        mRobot.pos_x = x_pos-mRobot.originPos_x;
        mRobot.pos_y = y_pos-mRobot.originPos_y; 

    } //if(m_pcanMsg_listen.ID == 0x1A1)

    if(m_pcanMsg_listen.ID == 0x1A2){
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl; 

    }


    //Z Position
    else if(m_pcanMsg_listen.ID == 0x1C1){
                
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
        z_pos = unwrap(mRobot.pos_z_pre_unwrap,z_pos);
                
        mRobot.pos_z = z_pos-mRobot.originPos_z;
        mRobot.pos_z_pre_unwrap = mRobot.pos_z; // unwrap

        
        //mRobot.pos_z = z_pos;

        //  if(mRobot.pos_z>=180 && mRobot.pos_z<=360){
        //      mRobot.pos_z=mRobot.pos_z-360;
        //       std::cout <<  "Angle 180~360 convert to -180~0" << std::endl;
        //       std::cout <<  "Angle 180~360 convert to -180~0" << std::endl;
        //       std::cout <<  "Angle 180~360 convert to -180~0" << std::endl;
        // }

        // else if(mRobot.pos_z>=-360 && mRobot.pos_z<=-180){
        //      mRobot.pos_z=mRobot.pos_z+360;
        //       std::cout <<  "Angle -180~-360 convert to 180~0" << std::endl;
        //       std::cout <<  "Angle -180~-360 convert to 180~0" << std::endl;
        //       std::cout <<  "Angle -180~-360 convert to 180~0" << std::endl;
        // }
        
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
            std::cout <<  "initOrigin Z Pos: " << z_pos << std::endl;
            z_pos=unwrap(0,z_pos);
            
            mRobot.originPos_z = z_pos;
            mRobot.pos_z_pre_unwrap = mRobot.originPos_z;
            mRobot.initZPosButton=true;   

            

        if(mRobot.initXYPosButton==true && mRobot.initZPosButton==true){
            mRobot.initPosButton =true;
            std::cout <<  std::endl;
            std::cout <<  std::endl;
            std::cout <<  "initOrigin X Pos: " << mRobot.originPos_x <<std::endl;
            std::cout <<  "initOrigin Y Pos: " << mRobot.originPos_y <<std::endl;
            std::cout <<  "initOrigin Z Pos_unwrap: " << mRobot.originPos_z << std::endl;   
            std::cout <<  std::endl;
        }

        }

    }         
    
 
}

