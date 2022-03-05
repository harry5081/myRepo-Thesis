#include <iostream>
#include "run.h"
#include <thread>
#include <string>

//#include "pid.h"
#include "controlPlant.h"
#include "readWritePlot.h"
#include "leader.h"






/////////////////////////////////////////////////////////////////////////////////////////////////
// leader
//ReadWritePlot *leader_Plot_time = new ReadWritePlot;
ReadWritePlot *leader_Plot_PosX = new ReadWritePlot;
ReadWritePlot *leader_Plot_PosY = new ReadWritePlot;

// planner
//ReadWritePlot *leader_Plot_time = new ReadWritePlot;
ReadWritePlot *planner_Plot_PosX = new ReadWritePlot;
ReadWritePlot *planner_Plot_PosY = new ReadWritePlot;





// ref
ReadWritePlot *ref_Plot_Vel_time = new ReadWritePlot;
ReadWritePlot *ref_Plot_Pos_time = new ReadWritePlot;

// ref forward speed
ReadWritePlot *ref_Plot_fspeed = new ReadWritePlot;
ReadWritePlot *ref_Plot_fsAngle = new ReadWritePlot;

// robot forward speed
ReadWritePlot *robot_Plot_fspeed = new ReadWritePlot;
ReadWritePlot *robot_Plot_fsAngle = new ReadWritePlot;


// plot ref_X_Vel
ReadWritePlot *ref_Plot_VelX= new ReadWritePlot;
// plot ref_Y_Vel
ReadWritePlot *ref_Plot_VelY= new ReadWritePlot;
// plot ref_Z_Vel
ReadWritePlot *ref_Plot_VelZ= new ReadWritePlot;

// plot ref_X_Pos
ReadWritePlot *ref_Plot_PosX= new ReadWritePlot;
// plot ref_Y_Pos
ReadWritePlot *ref_Plot_PosY= new ReadWritePlot;
// plot ref_Z_Pos
ReadWritePlot *ref_Plot_PosZ= new ReadWritePlot;



// plot demand_TIME
ReadWritePlot *vd_Plot_Vel_time = new ReadWritePlot;
ReadWritePlot *pd_Plot_Pos_time = new ReadWritePlot;

// plot demand_X_Vel
ReadWritePlot *vd_Plot_VelX= new ReadWritePlot;
// plot demand_Y_Vel
ReadWritePlot *vd_Plot_VelY= new ReadWritePlot;
// plot demand_Z_Vel
ReadWritePlot *vd_Plot_VelZ= new ReadWritePlot;

ReadWritePlot *fsd_Vel_Plot= new ReadWritePlot;
ReadWritePlot *fsd_Angle_Plot= new ReadWritePlot;


// plot demand_X_Pos
ReadWritePlot *pd_Plot_PosX= new ReadWritePlot;
// plot demand_Y_Pos
ReadWritePlot *pd_Plot_PosY= new ReadWritePlot;
// plot demand_Z_Pos
ReadWritePlot *pd_Plot_PosZ= new ReadWritePlot;




//CAN READ, CAN WRITE TIME
ReadWritePlot *canWriteTimePlot_X = new ReadWritePlot;
ReadWritePlot *canReadTimePlot_X = new ReadWritePlot;

// plot Vel_X
ReadWritePlot *canWriteDataPlot_X = new ReadWritePlot;
ReadWritePlot *canReadDataPlot_X = new ReadWritePlot;

// plot Vel_Y
ReadWritePlot *canWriteDataPlot_Y = new ReadWritePlot;
ReadWritePlot *canReadDataPlot_Y = new ReadWritePlot;


// plot Vel_Z
ReadWritePlot *canWriteDataPlot_Z = new ReadWritePlot;
ReadWritePlot *canReadDataPlot_Z = new ReadWritePlot;


// plot Pos_TIME
ReadWritePlot *canReadTimePlot_PosX = new ReadWritePlot;
ReadWritePlot *canReadTimePlot_PosZ = new ReadWritePlot;

// plot Pos_X
ReadWritePlot *canReadDataPlot_PosX= new ReadWritePlot;
// plot Pos_Y
ReadWritePlot *canReadDataPlot_PosY= new ReadWritePlot;
// plot Pos_Z
ReadWritePlot *canReadDataPlot_PosZ= new ReadWritePlot;

// plot Pos_X_correct
ReadWritePlot *canReadDataPlot_PosX_cor = new ReadWritePlot;
// plot Pos_Y_correct
ReadWritePlot *canReadDataPlot_PosY_cor = new ReadWritePlot;
/////////////////////////////////////////////////////////////////////////////////////////////////


run *m_run;
LEADER leader;
void virtualLeader();

void start();
void listen();

void writeDatatoFile();
void plotLeaderTraject();




//std::shared_ptr<std::thread> start_thread;
int main(int argc, char **argv) {

    
    std::cout << "Hello, world!..." << std::endl;
    //initDemand();
    
    
    
    
    std::thread leader_thread(&virtualLeader);
    m_run = new run();
        
    std::thread listen_thread(&listen);
    std::thread start_thread(&start);
    std::thread plot_thread(&writeDatatoFile);
    
   

    
    plot_thread.join();
    start_thread.join();
    listen_thread.join();
    leader_thread.join();
    //start_thread.reset(new std::thread(&start));
    //start_thread->join();

    
    
    return 0;
}



void virtualLeader(){

    std::cout << "Hello leader" << std::endl;
    

   
    //leader.cir_traject_init();
    leader.file_traject_init();
    
    std::thread leader_plot(&plotLeaderTraject);
    
    leader_plot.join();
    
    
    
}

void  start()
{   
    pybind11::scoped_interpreter guard{};
    while(1)
    {
        m_run->planner.s=leader.s_current; // for dynamic traj from leader
        m_run->start();
       
        
        
    }
}

void listen()
{
    std::cout << "Start Listening to CAN" << std::endl;

    //

    // while(1)
    // {
        m_run->canReadData();
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
            i_static_traject++;
            usleep(20000);
            
         }

         

    }
    

    
    // online dynamic trajectory
    else{
        while(1)
        {
        
        //leader_Plot_time->writeDatatoFile(time, "plot/       ");
        leader_Plot_PosX->writeDatatoFile(leader.xs, "plot/0_Leader_posx");
        leader_Plot_PosY->writeDatatoFile(leader.ys, "plot/0_Leader_posy");
        usleep(20000);

        }

    }

    
    

}


void writeDatatoFile()
{
    std::cout << "Start write Data to File" << std::endl;

    while(1)
    {
        float time = (float)clock()/CLOCKS_PER_SEC;

        // for Global Map
        for(int i=0;i<m_run->planner.pos_ref.size();i++){
            
            planner_Plot_PosX->writeDatatoFile(m_run->planner.pos_ref[i][0], "plot/1_Planner_posx");
            planner_Plot_PosY->writeDatatoFile(m_run->planner.pos_ref[i][1], "plot/1_Planner_posy");

        }

        

        planner_Plot_PosX->writeFromEnd=false;
        planner_Plot_PosY->writeFromEnd=false;


        


        
        //ref
        ref_Plot_Vel_time->writeDatatoFile(time, "plot/VelRef_Time");
        
        ref_Plot_VelX->writeDatatoFile(m_run->mpc.x_vel_ref, "plot/VelRef_Data_X");
        ref_Plot_VelY->writeDatatoFile(m_run->mpc.y_vel_ref, "plot/VelRef_Data_Y");
        ref_Plot_VelZ->writeDatatoFile(m_run->mpc.z_vel_ref, "plot/VelRef_Data_Z");

        ref_Plot_fspeed->writeDatatoFile(m_run->mpc.fspeedVel_ref, "plot/2_Fspeed_ref");
        ref_Plot_fsAngle->writeDatatoFile(m_run->mpc.fsAngle_ref, "plot/2_FsAngle_ref");

        ref_Plot_Pos_time->writeDatatoFile(time, "plot/PosRef_Time");
        ref_Plot_PosX->writeDatatoFile(m_run->mpc.x_pos_ref, "plot/PosRef_Data_X");
        ref_Plot_PosY->writeDatatoFile(m_run->mpc.y_pos_ref, "plot/PosRef_Data_Y");
        ref_Plot_PosZ->writeDatatoFile(m_run->mpc.z_pos_ref, "plot/PosRef_Data_Z");

        


        // // vd  pd
        vd_Plot_Vel_time->writeDatatoFile(time, "plot/VelDemand_Time");
        vd_Plot_VelX->writeDatatoFile(m_run->mRobot.vd_x, "plot/VelDemand_Data_X");
        vd_Plot_VelY->writeDatatoFile(m_run->mRobot.vd_y, "plot/VelDemand_Data_Y");
        vd_Plot_VelZ->writeDatatoFile(m_run->mRobot.vd_z, "plot/VelDemand_Data_Z");

        fsd_Vel_Plot->writeDatatoFile(m_run->mpc.fspeedVel_demand, "plot/3_Fspeed_demand");
        fsd_Angle_Plot->writeDatatoFile(m_run->mpc.fsAngle_demand, "plot/3_FsAngle_demand");

        pd_Plot_Pos_time->writeDatatoFile(time, "plot/PosDemand_Time");
        pd_Plot_PosX->writeDatatoFile(m_run->mRobot.pd_x, "plot/PosDemand_Data_X");
        pd_Plot_PosY->writeDatatoFile(m_run->mRobot.pd_y, "plot/PosDemand_Data_Y");
        pd_Plot_PosZ->writeDatatoFile(m_run->mRobot.pd_z, "plot/PosDemand_Data_Z");

        
        
        // std::cout << m_run->mRobot.ref_pos_x << std::endl;



        // controller input
        canWriteTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/CAN_Write_Time");
        canWriteDataPlot_X->writeDatatoFile(m_run->mRobot.controlInput_x_vel, "plot/CAN_Write_Data_X");
    
        //canWriteTimePlot_Y->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "CAN_Write_Time_Y");
        canWriteDataPlot_Y->writeDatatoFile(m_run->mRobot.controlInput_y_vel, "plot/CAN_Write_Data_Y");

        //canWriteTimePlot_Z->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "CAN_Write_Time_Z");
        canWriteDataPlot_Z->writeDatatoFile(m_run->mRobot.controlInput_z_vel, "plot/CAN_Write_Data_Z");
        


        // vel feedback
        canReadTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/CAN_Read_Time");
            
            canReadDataPlot_X->writeDatatoFile(m_run->mRobot.vel_x, "plot/CAN_Read_Data_X");
            canReadDataPlot_Y->writeDatatoFile(m_run->mRobot.vel_y, "plot/CAN_Read_Data_Y");
            canReadDataPlot_Z->writeDatatoFile(m_run->mRobot.vel_z, "plot/CAN_Read_Data_Z");


        // vel forward speed
        robot_Plot_fspeed->writeDatatoFile(m_run->mRobot.fspeedVel, "plot/9_Fspeed_robot");
        //robot_Plot_fsAngle->writeDatatoFile(m_run->mRobot.fsAngle_world, "plot/9_FsAngle_robot");
        robot_Plot_fsAngle->writeDatatoFile(m_run->mRobot.fsAngle, "plot/9_FsAngle_robot");
        




        // pos feedback

        canReadTimePlot_PosX->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/CAN_Read_Time_Pos");    
                
          

             canReadDataPlot_PosX->writeDatatoFile(m_run->mRobot.pos_x, "plot/CAN_Read_Data_PosX");
             canReadDataPlot_PosY->writeDatatoFile(m_run->mRobot.pos_y, "plot/CAN_Read_Data_PosY");

            canReadDataPlot_PosX_cor->writeDatatoFile(m_run->mRobot.pos_x_correct, "plot/CAN_Read_Data_PosX_Correct");
            canReadDataPlot_PosY_cor->writeDatatoFile(m_run->mRobot.pos_y_correct, "plot/CAN_Read_Data_PosY_Correct");

            canReadTimePlot_PosZ->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/CAN_Read_Time_PosZ");   
            canReadDataPlot_PosZ->writeDatatoFile(m_run->mRobot.pos_z, "plot/CAN_Read_Data_PosZ");

        

        
        
            usleep(20000);
        
        
    }
    
}






