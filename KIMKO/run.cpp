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

#define PI 3.14159265

// timer
 float timeDiff;
 float timeTemp;
 auto pre_chrono_time = std::chrono::high_resolution_clock::now();

// tracking and obs avoidance
 enum BEHAVIOR {TRACK=0, AVOID};
 BEHAVIOR behavior=TRACK;
 float epsilon = 500;


// pid
PID pid_x(3,0,1,0.3);
PID pid_y(3,0,1,0.3);
PID pid_z(2,0,1,0.1);

// PID pid_x(0,0,1,0);
// PID pid_y(0,0,1,0);
// PID pid_z(0,0,1,0);



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
m_int16_velocity_level2(0),
//obs_run_A(-750,-300,500)
//obs_run_A(-1500,3000,10), // no obs
//obs_run_A(-1500,0,700),
obs_run_A(-4000,0,300),
mpc(planner.window)
{
    
    while(CAN_Initialize(m_Channel, m_Btr0Btr1) != PCAN_ERROR_OK)
    {
        sleep(1);
        printf("pcan init fails.. \n");
        
    }
  
    printf("pcan init success.. \n");
    
    init();
    initOriginPos(); // try to reset the register in robot, so dont have to switch it off every time
                     // now this function is not working due to correction of sensor position



}
void run::start()
{
    ////////////////////////////////   timer to print out sampling time   ////////////////////////////////////////////////////////
    auto chronoTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> chronoDiff_ms = chronoTime - pre_chrono_time;
    std::cout << "!!!!!!!!!!!!!!!!!!!  ChronoTime  !!!!!!!!!!!!!!!!  Sampling:  "<< chronoDiff_ms.count() << " ms " <<std::endl;
    pre_chrono_time=chronoTime;

    std::this_thread::sleep_for(std::chrono::milliseconds (200-int(mpc.mpcExTime)));
   
   
    ////////////////////////////////////////////   Manual or Automatic   ////////////////////////////////////////////////////////
    m_int16_operatingmode = 2;          // automatic mode

    // m_int16_operatingmode = 1;       // manual mode
    //printf("enter the operating mode");
    //scanf("%hd", &m_int16_operatingmode);


    ////////////////////////////////////////////   Obstacle Avoidance      ////////////////////////////////////////////////////////
    std::vector<std::vector<int>> obs_data;
    obs_data.push_back(obs_run_A.data);

    
    ////////////////////////////////////////////   Local Planner (Green Reference)   ////////////////////////////////////////////////////////
    
    //planner.cir_traject_TNB();        //  circular path
    planner.traject_from_file();        //  long path throughout the lab

    mRobot.fsAngle_fromRef_360 = planner.pos_ref[0][2]*180.0/PI; //correct robot fsAngle with respect to reference
    mRobot.calFspeed(); // calculate forward speed and angle of forward speed

    
    /////////////////////////////      Data from Planner module to MPC module       ////////////////////////////////////
    std::vector<std::vector<float>> p_ref_dyn = planner.pos_ref; // [x1, y1, psi_1, x2, y2, psi_2, ...]
    std::vector<std::vector<float>> v_ref_dyn = planner.vel_ref; // [s_dot1, 0, ws_1, s_dot2, 0, ws_2, ...]
        
    std::vector<std::vector<float>> ori_ref_dyn = planner.ori_ref;
    std::vector<std::vector<float>> guess = planner.guess;      // inital guess for MPC in CasADi
   
    std::vector<float> p_init = {mRobot.pos_x_correct, mRobot.pos_y_correct, mRobot.fsAngle_rad};
    //std::vector<float> p_init = {mRobot.pos_x_correct, mRobot.pos_y_correct, float(mRobot.fsAngle_360*PI/180.0)};
    std::vector<float> v_init = {mRobot.fspeedVel, 0, 0};
   
    std::vector<float> ori_init = {mRobot.pos_z, mRobot.vel_z}; 

    mpc.window_planner=planner.window;


    ///////////////////////////////////////      MPC Execution       ///////////////////////////////////////////
    
    //mpc.mpcAvoid_obsData_presol(obs_data, p_ref_dyn, v_ref_dyn, p_init, v_init, ori_ref_dyn, ori_init, guess);
    //mpc.mpcErrDyn_xy_plotPredicHorz_presol(p_ref_dyn, v_ref_dyn, p_init, v_init, ori_ref_dyn, ori_init, guess);

    //////////////////////////////////  MPC Execution  + Obstacle Avoidance  ///////////////////////////////////////
    float distance = sqrt(pow((mRobot.pos_x_correct-obs_run_A.obs_x),2)+pow((mRobot.pos_y_correct-obs_run_A.obs_y),2));
    if(distance<=700+epsilon){
        behavior=AVOID;
    }
    else if(distance>700+1.5*epsilon){
        behavior=TRACK;
    }

    if(behavior == AVOID){
         mpc.mpcAvoid_obsData_presol(obs_data, p_ref_dyn, v_ref_dyn, p_init, v_init, ori_ref_dyn, ori_init, guess);
    }
   
    else{
        mpc.mpcErrDyn_xy_plotPredicHorz_presol(p_ref_dyn, v_ref_dyn, p_init, v_init, ori_ref_dyn, ori_init, guess);
    }


    mpc.x_vel_demand=mpc.fspeedVel_demand*cos(mpc.fsAngle_demand_rad-mRobot.pos_z_rad);
    mpc.y_vel_demand=mpc.fspeedVel_demand*sin(mpc.fsAngle_demand_rad-mRobot.pos_z_rad);

   /////////////////////////////////////////////  Plot Green Reference  ///////////////////////////////////////////////

    mpc.x_pos_ref = planner.pos_ref[0][0]; //plot ref green
    mpc.y_pos_ref = planner.pos_ref[0][1]; //plot ref green

    mpc.fsAngle_ref = planner.pos_ref[0][2]*180.0/PI; //plot ref green
    mpc.fspeedVel_ref = planner.vel_ref[0][0]; //plot ref green
    
    mpc.z_pos_ref = planner.ori_ref[0][0]; //plot ref green
    mpc.z_vel_ref = planner.ori_ref[0][1]; //plot ref green    

   

    mpc.x_vel_ref = mpc.fspeedVel_ref*cos(planner.pos_ref[0][2]-planner.ori_ref[0][0]*PI/180.0); //plot ref green
    mpc.y_vel_ref = mpc.fspeedVel_ref*sin(planner.pos_ref[0][2]-planner.ori_ref[0][0]*PI/180.0); //plot ref green


    /////////////////////////////      Data from MPC module to PID module       ////////////////////////////////////
    //  X
    mRobot.vd_x = mpc.x_vel_demand;
    mRobot.pd_x = mpc.x_pos_demand;
    
    // Y 
    mRobot.vd_y = mpc.y_vel_demand;
    mRobot.pd_y = mpc.y_pos_demand;

    // Z
    mRobot.vd_z = mpc.z_vel_demand;
    mRobot.pd_z = mpc.z_pos_demand;


    /////////////////////////////      PID Tuning       ////////////////////////////////////
    //  X
    // mRobot.pd_x = mpc.sinePosDemand(time);
    // mRobot.vd_x = mpc.cosVelDemand(time);

    //mRobot.pd_x = mpc.stepPosDemand(time);
    //mRobot.vd_x = mpc.stepVelDemand(time);

    // mRobot.pd_x = (1)*mpc.sineToTenPosDemand(time);
    // mRobot.vd_x = (1)*mpc.cosToTenVelDemand(time);

    // mRobot.pd_x = mpc.powThreePosDemand(time);
    // mRobot.vd_x = mpc.powTwoVelDemand(time);

    // mRobot.pd_x = mpc.sineToTenPosDemand(time);
    // mRobot.vd_x = mpc.cosToTenVelDemand(time);

    //mRobot.pd_x = 0;
    // mRobot.vd_x = 0;
    

    //  Y
    //mRobot.pd_y = mpc.sinePosDemand(time);
    //mRobot.vd_y = mpc.cosVelDemand(time);

    // mRobot.pd_y = mpc.powThreePosDemand(time);
    // mRobot.vd_y = mpc.powTwoVelDemand(time);

    //mRobot.pd_y = mpc.sineToTenPosDemand(time);
    //mRobot.vd_y = mpc.cosToTenVelDemand(time);

    // mRobot.pd_y = 0;
    // mRobot.vd_y = 0;


    //  Z
   //mRobot.pd_z = mpc.sinePosDemand(time)/10;
    //mRobot.vd_z = mpc.cosVelDemand(time)/10;
    
    //mRobot.pd_z = mpc.sineToTenPosDemand(time)/10;
    //mRobot.vd_z = mpc.cosToTenVelDemand(time)/10;

    // mRobot.vd_z =0;
    // mRobot.pd_z = 0;


    ///////////////////////////////////////      PID Execution       ///////////////////////////////////////////
    
    m_int16_desired_velocity_X = pid_x.pidExe(mRobot.pd_x-mRobot.pos_x_correct, mRobot.vd_x, mRobot.vd_x-mRobot.vel_x);
    m_int16_desired_velocity_Y = pid_y.pidExe(mRobot.pd_y-mRobot.pos_y_correct, mRobot.vd_y, mRobot.vd_y-mRobot.vel_y);
    m_f_desired_velocity_Z = pid_z.pidExeAngle(mRobot.pd_z-mRobot.pos_z,mRobot.vd_z,mRobot.vd_z-mRobot.vel_z);
  
    // open loop response
    // m_int16_desired_velocity_X = mpc.stepVelDemand(time);
    // m_int16_desired_velocity_Y = 0;
    // m_f_desired_velocity_Z = 0;

    ///////////////////////////////////////      Manual Mode Input       ///////////////////////////////////////////

     
    // printf("enter the X velocity speed :");
    // scanf("%hd", &m_int16_desired_velocity_X);

    // printf("enter the Y velocity speed :");
    // scanf("%hd", &m_int16_desired_velocity_Y);

    // printf("enter the Z velocity speed :");
    // scanf("%f", &m_f_desired_velocity_Z);

    

  ///////////////////////////////////////      Controller Input       ///////////////////////////////////////////

    mRobot.controlInput_x_vel = m_int16_desired_velocity_X;
    mRobot.controlInput_y_vel = m_int16_desired_velocity_Y;
    mRobot.controlInput_z_vel = m_f_desired_velocity_Z;

  
    
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

        usleep(20000);  //20 ms

        
        
        
    }
}

void run::canReadData(){

    TPCANStatus result;
       
   while(1){
 
        result = CAN_Read(m_Channel, &m_pcanMsg_listen, nullptr);
                
        if(result != PCAN_ERROR_QRCVEMPTY){
            
            getVelocityValue();
            getPositionValue();
  
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

        
        // if(draw == PLOT){
        //     // canReadTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/CAN_Read_Time");
            
        //     // canReadDataPlot_X->writeDatatoFile(mRobot.vel_x, "plot/CAN_Read_Data_X");
        //     // canReadDataPlot_Y->writeDatatoFile(mRobot.vel_y, "plot/CAN_Read_Data_Y");
        //     // canReadDataPlot_Z->writeDatatoFile(mRobot.vel_z, "plot/CAN_Read_Data_Z");
            
        // }
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
       
       
        // correct the wrong sensor value of position
        mRobot.pos_x_correct = mRobot.pos_x_correct_pre+(cos(2.0*mRobot.pos_z_rad)*(x_pos-mRobot.pos_x_raw_pre) + (-1.0)*sin(2.0*mRobot.pos_z_rad)*(y_pos-mRobot.pos_y_raw_pre));
        mRobot.pos_y_correct = mRobot.pos_y_correct_pre+(sin(2.0*mRobot.pos_z_rad)*(x_pos-mRobot.pos_x_raw_pre) + cos(2.0*mRobot.pos_z_rad)*(y_pos-mRobot.pos_y_raw_pre));

        mRobot.pos_x_raw_pre = x_pos;
        mRobot.pos_y_raw_pre = y_pos;
        
        mRobot.pos_x_correct_pre=mRobot.pos_x_correct;
        mRobot.pos_y_correct_pre=mRobot.pos_y_correct;

        mRobot.pos_x = x_pos; //plot raw wrong position data
        mRobot.pos_y = y_pos;   


        // mRobot.pos_x = x_pos-mRobot.originPos_x;
        // mRobot.pos_y = y_pos-mRobot.originPos_y; 

        // mRobot.pos_x_correct = mRobot.pos_x_correct_pre+(cos(2.0*mRobot.pos_z_rad)*(x_pos-mRobot.pos_x_raw_pre) + (-1.0)*sin(2.0*mRobot.pos_z_rad)*(y_pos-mRobot.pos_y_raw_pre));
        // mRobot.pos_y_correct = mRobot.pos_y_correct_pre+(sin(2.0*mRobot.pos_z_rad)*(x_pos-mRobot.pos_x_raw_pre) + cos(2.0*mRobot.pos_z_rad)*(y_pos-mRobot.pos_y_raw_pre));

        // mRobot.pos_x_raw_pre = mRobot.pos_x;
        // mRobot.pos_y_raw_pre = mRobot.pos_y;

        // mRobot.pos_x_correct_pre=mRobot.pos_x_correct;
        // mRobot.pos_y_correct_pre=mRobot.pos_y_correct;

    } //if(m_pcanMsg_listen.ID == 0x1A1)

    
    //Z Position
    else if(m_pcanMsg_listen.ID == 0x1C1){
                       
        uint32_t z_pos_temp=static_cast<uint32_t>(m_pcanMsg_listen.DATA[0])|static_cast<uint32_t>(m_pcanMsg_listen.DATA[1]<<8)\
                        |static_cast<uint32_t>(m_pcanMsg_listen.DATA[2]<<16)|static_cast<uint32_t>(m_pcanMsg_listen.DATA[3]<<24);
        float z_pos = *((float *) &z_pos_temp);
        
        mRobot.pos_z_raw = z_pos;
        z_pos = unwrap(mRobot.pos_z_pre_unwrap,z_pos); // make z pos continuous
                
        mRobot.pos_z = z_pos-mRobot.originPos_z;
        mRobot.pos_z_pre_unwrap = mRobot.pos_z; // unwrap
        mRobot.pos_z_rad = mRobot.pos_z * PI/180.0;
        
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

