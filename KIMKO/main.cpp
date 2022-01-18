#include <iostream>
#include "run.h"
#include <thread>

//#include "pid.h"
#include "controlPlant.h"
#include "readWritePlot.h"






/////////////////////////////////////////////////////////////////////////////////////////////////
// ref
ReadWritePlot *ref_Plot_Vel_time = new ReadWritePlot;
ReadWritePlot *ref_Plot_Pos_time = new ReadWritePlot;

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
void start();
void listen();

void writeDatatoFile();


//std::shared_ptr<std::thread> start_thread;
int main(int argc, char **argv) {

    
        
    std::cout << "Hello, world!..." << std::endl;
    //initDemand();
      
    m_run = new run();

    std::thread start_thread(&start);
        
    std::thread listen_thread(&listen);
    
    

    std::thread plot_thread(&writeDatatoFile);
   


    plot_thread.join();
    start_thread.join();
    listen_thread.join();

    //start_thread.reset(new std::thread(&start));
    //start_thread->join();

    
    
    return 0;
}

void  start()
{   
    pybind11::scoped_interpreter guard{};
    while(1)
    {
        m_run->start();
        
    }
}

void listen()
{
    std::cout << "Start Listening to CAN" << std::endl;

    //sleep(3);

    while(1)
    {
        m_run->canReadData();
        
        
    }
    
}


void writeDatatoFile()
{
    std::cout << "Start write Data to File" << std::endl;

    while(1)
    {
        float time = (float)clock()/CLOCKS_PER_SEC;

        //ref
        ref_Plot_Vel_time->writeDatatoFile(time, "plot/VelRef_Time");
        ref_Plot_VelX->writeDatatoFile(m_run->mpc.x_vel_ref, "plot/VelRef_Data_X");
        ref_Plot_VelY->writeDatatoFile(m_run->mpc.y_vel_ref, "plot/VelRef_Data_Y");
        ref_Plot_VelZ->writeDatatoFile(m_run->mpc.z_vel_ref, "plot/VelRef_Data_Z");

        ref_Plot_Pos_time->writeDatatoFile(time, "plot/PosRef_Time");
        ref_Plot_PosX->writeDatatoFile(m_run->mpc.x_pos_ref, "plot/PosRef_Data_X");
        ref_Plot_PosY->writeDatatoFile(m_run->mpc.y_pos_ref, "plot/PosRef_Data_Y");
        ref_Plot_PosZ->writeDatatoFile(m_run->mpc.z_pos_ref, "plot/PosRef_Data_Z");


        // vd  pd
         vd_Plot_Vel_time->writeDatatoFile(time, "plot/VelDemand_Time");
         vd_Plot_VelX->writeDatatoFile(m_run->mRobot.vd_x, "plot/VelDemand_Data_X");
         vd_Plot_VelY->writeDatatoFile(m_run->mRobot.vd_y, "plot/VelDemand_Data_Y");
         vd_Plot_VelZ->writeDatatoFile(m_run->mRobot.vd_z, "plot/VelDemand_Data_Z");

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






