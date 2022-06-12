#include <iostream>
#include "run.h"
#include <thread>
#include <string>

//#include "pid.h"
#include "controlPlant.h"
#include "leader.h"
#include "obs.h"
#include "main.h"


run *m_run;
LEADER leader;


void start();
void listen();

void writeDatatoFile();
void plotLeaderTraject();
void plotObs();





//std::shared_ptr<std::thread> start_thread;
int main(int argc, char **argv) {

    
    std::cout << "Hello, world!..." << std::endl;
    m_run = new run();

    ////////////////////////////    Leader  ////////////////////////////////

    //leader.cir_traject_init(); // so far used for plotting out global path
    leader.file_traject_init();  // so far used for plotting out global path
    plotLeaderTraject();
    
    
    ////////////////////////////    Obstacle  ////////////////////////////////
    plotObs();


    ////////////////////////////    Run the Robot  ////////////////////////////////
    
    std::thread listen_thread(&listen);         // read the sensor values
    std::thread start_thread(&start);           // MPC + PID + CAN_Write controller input
    std::thread plot_thread(&writeDatatoFile); // output data to several .txt files for plotting
    
   

    ////////////////////////////  Finish the threads  ////////////////////////////////
    plot_thread.join();
    start_thread.join();
    listen_thread.join();
    

    
    
    return 0;
}


void  start()
{   
    pybind11::scoped_interpreter guard{};
    
    while(1)
    {
        m_run->planner.s = leader.s_current; // given the end of global path to planner module
        m_run->start();

    }
}

void listen()
{
    std::cout << "Start Listening to CAN" << std::endl;

    // while(1)
    // {
        m_run->canReadData(); // infinite loop
        //usleep(50000);
        
    // }
    
}



void plotLeaderTraject(){

    //all trajectory given in advance
    if(leader.period ==0){
        static int i_static_traject=0;
        
        while(i_static_traject<leader.point_cnt){
            
            leader_Plot_PosX->writeDatatoFile(leader.curve[i_static_traject][0], "plot/0_Leader_posx");
            leader_Plot_PosY->writeDatatoFile(leader.curve[i_static_traject][1], "plot/0_Leader_posy");

            //i_static_traject++;
            i_static_traject=i_static_traject+5;
        }
    }

    // online dynamic global trajectory
    // else{
    //     while(1)
    //     {
        
    //     //leader_Plot_time->writeDatatoFile(time, "plot/       ");
    //     leader_Plot_PosX->writeDatatoFile(leader.xs, "plot/0_Leader_posx");
    //     leader_Plot_PosY->writeDatatoFile(leader.ys, "plot/0_Leader_posy");
    //     usleep(20000);

    //     }

    // }
}

void plotObs(){
    
    obs_Plot_PosX->writeDatatoFile(m_run->obs_run_A.obs_x, "plot/o_1_obstacle_posx");
    obs_Plot_PosY->writeDatatoFile(m_run->obs_run_A.obs_y, "plot/o_1_obstacle_posy");

    for(int i=0;i<m_run->obs_run_A.obs_bound_x.size();i++){
            
            obs_Plot_PosX->writeDatatoFile(m_run->obs_run_A.obs_bound_x[i], "plot/o_1_obstacle_posx");
            obs_Plot_PosY->writeDatatoFile(m_run->obs_run_A.obs_bound_y[i], "plot/o_1_obstacle_posy");
    }
}


void writeDatatoFile()
{
    std::cout << "Start write Data to File" << std::endl;

    while(1)
    {
        float time = (float)clock()/CLOCKS_PER_SEC;

        //////////////////////// for Global Map  //////////////////////////////////////////////////
        // the moving reference green color
        for(int i=0;i<m_run->planner.pos_ref.size();i++){
            planner_Plot_PosX->writeDatatoFile(m_run->planner.pos_ref[i][0], "plot/1_Planner_posx");
            planner_Plot_PosY->writeDatatoFile(m_run->planner.pos_ref[i][1], "plot/1_Planner_posy");
        }
        
        planner_Plot_PosX->writeFromEnd=false;
        planner_Plot_PosY->writeFromEnd=false;

        // prediction horz blue color
        for(int i=0;i<m_run->mpc.predictHorz.size();i++){
            
            predicHorz_PosX->writeDatatoFile(m_run->mpc.predictHorz[i][0], "plot/1_predicHorz_posx");
            predicHorz_PosY->writeDatatoFile(m_run->mpc.predictHorz[i][1], "plot/1_predicHorz_posy");
        }
        predicHorz_PosX->writeFromEnd=false;
        predicHorz_PosY->writeFromEnd=false;
        //////////////////////// for Global Map  //////////////////////////////////////////////////


        //////////////////////// for other plotting  //////////////////////////////////////////////
        //  2_   reference green color
        ref_Plot_Vel_time->writeDatatoFile(time, "plot/2_VelRef_Time");
        ref_Plot_VelX->writeDatatoFile(m_run->mpc.x_vel_ref, "plot/2_VelRef_Data_X");
        ref_Plot_VelY->writeDatatoFile(m_run->mpc.y_vel_ref, "plot/2_VelRef_Data_Y");
        ref_Plot_VelZ->writeDatatoFile(m_run->mpc.z_vel_ref, "plot/2_VelRef_Data_Z");

        ref_Plot_fspeed->writeDatatoFile(m_run->mpc.fspeedVel_ref, "plot/2_Fspeed_ref");
        ref_Plot_fsAngle->writeDatatoFile(m_run->mpc.fsAngle_ref, "plot/2_FsAngle_ref");

        ref_Plot_Pos_time->writeDatatoFile(time, "plot/2_PosRef_Time");
        ref_Plot_PosX->writeDatatoFile(m_run->mpc.x_pos_ref, "plot/2_PosRef_Data_X");
        ref_Plot_PosY->writeDatatoFile(m_run->mpc.y_pos_ref, "plot/2_PosRef_Data_Y");
        ref_Plot_PosZ->writeDatatoFile(m_run->mpc.z_pos_ref, "plot/2_PosRef_Data_Z");


        //  3_  desire velocite and position
        vd_Plot_Vel_time->writeDatatoFile(time, "plot/3_VelDemand_Time");
        vd_Plot_VelX->writeDatatoFile(m_run->mRobot.vd_x, "plot/3_VelDemand_Data_X");
        vd_Plot_VelY->writeDatatoFile(m_run->mRobot.vd_y, "plot/3_VelDemand_Data_Y");
        vd_Plot_VelZ->writeDatatoFile(m_run->mRobot.vd_z, "plot/3_VelDemand_Data_Z");

        fsd_Vel_Plot->writeDatatoFile(m_run->mpc.fspeedVel_demand, "plot/3_Fspeed_demand");
        fsd_Angle_Plot->writeDatatoFile(m_run->mpc.fsAngle_demand, "plot/3_FsAngle_demand");

        pd_Plot_Pos_time->writeDatatoFile(time, "plot/3_PosDemand_Time");
        pd_Plot_PosX->writeDatatoFile(m_run->mRobot.pd_x, "plot/3_PosDemand_Data_X");
        pd_Plot_PosY->writeDatatoFile(m_run->mRobot.pd_y, "plot/3_PosDemand_Data_Y");
        pd_Plot_PosZ->writeDatatoFile(m_run->mRobot.pd_z, "plot/3_PosDemand_Data_Z");

        //   4_  controller input
        canWriteTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/4_CAN_Write_Time");
        canWriteDataPlot_X->writeDatatoFile(m_run->mRobot.controlInput_x_vel, "plot/4_CAN_Write_Data_X");
        canWriteDataPlot_Y->writeDatatoFile(m_run->mRobot.controlInput_y_vel, "plot/4_CAN_Write_Data_Y");
        canWriteDataPlot_Z->writeDatatoFile(m_run->mRobot.controlInput_z_vel, "plot/4_CAN_Write_Data_Z");

        //   5_  velocity from sensor
        canReadTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/5_CAN_Read_Time");
        canReadDataPlot_X->writeDatatoFile(m_run->mRobot.vel_x, "plot/5_CAN_Read_Data_X");
        canReadDataPlot_Y->writeDatatoFile(m_run->mRobot.vel_y, "plot/5_CAN_Read_Data_Y");
        canReadDataPlot_Z->writeDatatoFile(m_run->mRobot.vel_z, "plot/5_CAN_Read_Data_Z");

        //   5_  vel forward speed
        robot_Plot_fspeed->writeDatatoFile(m_run->mRobot.fspeedVel, "plot/5_Fspeed_robot");
        //robot_Plot_fsAngle->writeDatatoFile(m_run->mRobot.fsAngle_world, "plot/9_FsAngle_robot");
        robot_Plot_fsAngle->writeDatatoFile(m_run->mRobot.fsAngle_360, "plot/5_FsAngle_robot");
        
        //   5_  position from sensor
        canReadTimePlot_PosX->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/5_CAN_Read_Time_Pos");  

        //   5_  origianl position from sensor is wrong, we have correct them
        canReadDataPlot_PosX->writeDatatoFile(m_run->mRobot.pos_x, "plot/5_CAN_Read_Data_PosX"); 
        canReadDataPlot_PosY->writeDatatoFile(m_run->mRobot.pos_y, "plot/5_CAN_Read_Data_PosY");

        //   5_  the corrected position
        canReadDataPlot_PosX_cor->writeDatatoFile(m_run->mRobot.pos_x_correct, "plot/5_CAN_Read_Data_PosX_Correct");
        canReadDataPlot_PosY_cor->writeDatatoFile(m_run->mRobot.pos_y_correct, "plot/5_CAN_Read_Data_PosY_Correct");

        canReadTimePlot_PosZ->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/5_CAN_Read_Time_PosZ");   
        canReadDataPlot_PosZ->writeDatatoFile(m_run->mRobot.pos_z, "plot/5_CAN_Read_Data_PosZ");

        //////////////////////// for other plotting  //////////////////////////////////////////////
        
        // output the data to .txt every 20 ms
        std::this_thread::sleep_for(std::chrono::milliseconds (20)); 
            
            
            
        
    }
    
}






