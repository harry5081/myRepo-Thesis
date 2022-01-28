#include <fstream>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <math.h>
#include <cmath>
#include "../matplotlib-cpp-master/matplotlibcpp.h"

//g++ -o plot.out plotThreeDyn.cpp -I/usr/include/python2.7 /lib/x86_64-linux-gnu/libpthread.so.0 -lpython2.7


#define PI 3.14159265

template<class T> void readFileToVector(std::string fileName, std::vector<T> &data);
void windowSizeControl();
void plotX();
void plot(std::string CAN_Write_Time, std::string CAN_Write_Data, std::string CAN_Read_Time, std::string CAN_Read_Data);
void intp();
void plotTheta();

int wSize = 20;

std::vector<float> leader_posx_temp;
std::vector<float> leader_posy_temp;

std::vector<float> planner_posx_temp;
std::vector<float> planner_posy_temp;


std::vector<float> posRef_Time_temp;
std::vector<float> posRef_X_temp;
std::vector<float> posRef_Y_temp;
std::vector<float> posRef_Z_temp;

std::vector<float> velRef_Time_temp;
std::vector<float> velRef_X_temp;
std::vector<float> velRef_Y_temp;
std::vector<float> velRef_Z_temp;


std::vector<float> posDemand_Time_temp;
std::vector<float> posDemand_X_temp;
std::vector<float> posDemand_Y_temp;
std::vector<float> posDemand_Z_temp;

std::vector<float> velDemand_Time_temp;
std::vector<float> velDemand_X_temp;
std::vector<float> velDemand_Y_temp;
std::vector<float> velDemand_Z_temp;



std::vector<float> canWriteTime_temp;
std::vector<float> canReadTime_temp;


std::vector<float> canWriteData_temp;
std::vector<float> canReadData_temp;

std::vector<float> canWriteData_Y_temp;
std::vector<float> canReadData_Y_temp;

std::vector<float> canWriteData_Z_temp;
std::vector<float> canReadData_Z_temp;



std::vector<float> canReadTime_Pos_temp;
std::vector<float> canReadTime_PosZ_temp;
std::vector<float> canReadData_PosX_temp;
std::vector<float> canReadData_PosY_temp;
std::vector<float> canReadData_PosZ_temp;

std::vector<float> PosX_cor;
std::vector<float> PosY_cor;




bool intp_button = false;
bool show_angle_button = false;



//std::shared_ptr<std::thread> plotX_thread;
namespace plt = matplotlibcpp;

int main() {
    std::cout<<" Start Read File and Plot"<<std::endl;

    
    std::thread plot_thread(&plotX);
    std::thread intp_thread(&intp);
 
    
 
    plot_thread.join();
    intp_thread.join();

    
  
    return 0;
}

template<class T>
void readFileToVector(std::string fileName, std::vector<T> &dataVec){

    std::ifstream fileRead(fileName); 
    if(!fileRead.is_open()){
        std::cout << "Failed to open " << fileName << " for Read!\n";
        return;
    }

    T data;
    while (fileRead >> data){
        dataVec.push_back(data);
    }
 
    fileRead.close();


}

void windowSizeControl(){
       
    
       
        if(*(canWriteTime_temp.end()-1)>=10 && *(canWriteTime_temp.end()-1) >= wSize -10 ){
            wSize = int(*(canWriteTime_temp.end()-1)*2);
        }

        else if( *(canWriteTime_temp.end()-1)<10){

            wSize=20;
        }

        
    
 
}

void intp(){
    while(true){
        
        char temp;
    
        std::cout<<"Press 's' to stop updating the plot"<<std::endl;
        std::cout<<"Press 'c' to continue updating the plot"<<std::endl;

        std::cin >> temp;

        if(temp == 's'){
            intp_button = true;
            show_angle_button = false;
        }

        else if(temp == 'c'){
            intp_button = false;
            show_angle_button = false;
        }
        else if(temp == 'a'){
            show_angle_button = true;
            return;
        }
        
        

        sleep(1);
    }

    

}
    

void plotTheta(){
    
    readFileToVector("../build/plot/CAN_Read_Data_PosX_Correct", PosX_cor);
    readFileToVector("../build/plot/CAN_Read_Data_PosY_Correct", PosY_cor);
    readFileToVector("../build/plot/CAN_Read_Data_PosZ", canReadData_PosZ_temp);
    
    
    std::vector<float> xPoint;
    std::vector<float> yPoint;
    std::vector<float> xDir;
    std::vector<float> yDir;

    float posX_cur, posY_cur, posZ_cur;
    float posX_pre, posY_pre, posZ_pre;
    float disDiff;
    float angDiff;
    
    for(int i=0; i<canReadData_PosZ_temp.size(); i++){
        
        posX_cur = PosX_cor[i];
        posY_cur = PosY_cor[i];
        posZ_cur = canReadData_PosZ_temp[i];

        disDiff = sqrt(pow(posX_cur-posX_pre,2) + pow(posY_cur-posY_pre,2));
        angDiff = abs(posZ_cur-posZ_pre);

        if(i==0){
            xDir.push_back(1*cos(canReadData_PosZ_temp[i]* PI / 180.0));
            yDir.push_back(1*sin(canReadData_PosZ_temp[i]* PI / 180.0));
        
            xPoint.push_back(PosX_cor[i]);
            yPoint.push_back(PosY_cor[i]);

            posX_pre = posX_cur;
            posY_pre = posY_cur;
            posZ_pre = posZ_cur;

        }

        else if(angDiff>=3 || disDiff>50){
            xDir.push_back(1*cos(canReadData_PosZ_temp[i]* PI / 180.0));
            yDir.push_back(1*sin(canReadData_PosZ_temp[i]* PI / 180.0));
        
            xPoint.push_back(PosX_cor[i]);
            yPoint.push_back(PosY_cor[i]);

            posX_pre = posX_cur;
            posY_pre = posY_cur;
            posZ_pre = posZ_cur;

        }

        else if( i==canReadData_PosZ_temp.size()-1)
        {
            xDir.push_back(1*cos(canReadData_PosZ_temp[i]* PI / 180.0));
            yDir.push_back(1*sin(canReadData_PosZ_temp[i]* PI / 180.0));
        
            xPoint.push_back(PosX_cor[i]);
            yPoint.push_back(PosY_cor[i]);

        }

        

        
        
        
        

    }
    
    plt::figure(4);
    plt::grid();
    plt::xlim(-200, 200);
    plt::ylim(-200, 200);
    plt::plot(PosY_cor, PosX_cor);
    plt::quiver(yPoint, xPoint, yDir, xDir);
    plt::show();

}
void plotX(){
    
    ///////////////////////////     velocity part     ///////////////////////////
    //plt::figure_size(1200, 780);
    
    plt::figure(1);
    
    plt::subplot(3,1,1);
    plt::title("Vel");
    plt::Plot plot0_1("v_ref",velRef_Time_temp,velRef_X_temp,"g");
    plt::Plot plot0("v_d",velDemand_Time_temp,velDemand_X_temp,"b");  
    plt::Plot plot1("v_input",canWriteTime_temp,canWriteData_temp,"k");
    plt::Plot plot2("v_sensor",canReadTime_temp,canReadData_temp,"r");
    
    plt::grid();
    plt::legend();


    plt::subplot(3,1,2);
    plt::Plot plot2_8("v_ref_y",velRef_Time_temp,velRef_Y_temp,"g");
    plt::Plot plot2_9("VelDemandY",velDemand_Time_temp,velDemand_Y_temp,"b");
    plt::Plot plot3("WritecanY",canWriteTime_temp,canWriteData_Y_temp,"k");
    plt::Plot plot4("WritereadY",canReadTime_temp,canReadData_Y_temp,"r");
   
    plt::grid();


    plt::subplot(3,1,3);
    plt::Plot plot4_4("v_ref_z",velRef_Time_temp,velRef_Z_temp,"g");
    plt::Plot plot4_5("VelDemandZ",velDemand_Time_temp,velDemand_Z_temp,"b");
    plt::Plot plot4_6("WritecanZ",canWriteTime_temp,canWriteData_Z_temp,"k");
    plt::Plot plot4_7("WritereadZ",canReadTime_temp,canReadData_Z_temp,"r");
   
    plt::grid();
    



    ///////////////////////////     position part     ///////////////////////////
    plt::figure(2);
    
    plt::subplot(3,1,1);
    plt::title("Pos");
    plt::Plot plot5_1("p_ref",posRef_Time_temp,posRef_X_temp,"g");   
    plt::Plot plot5("p_d",posDemand_Time_temp,posDemand_X_temp,"b");
    plt::Plot plot6("p_sensor",canReadTime_Pos_temp,canReadData_PosX_temp,"r");
    plt::Plot plot6_1("p_correct",canReadTime_Pos_temp,PosX_cor,"y--");

    plt::grid(); 
    plt::legend();

    plt::subplot(3,1,2);
    plt::Plot plot7_1("p_ref_y",posRef_Time_temp,posRef_Y_temp,"g"); 
    plt::Plot plot7("PosDemandY",posDemand_Time_temp,posDemand_Y_temp,"b");
    plt::Plot plot8("WritereadPosY",canReadTime_Pos_temp,canReadData_PosY_temp,"r");
    plt::Plot plot8_1("p_Y_correct",canReadTime_Pos_temp,PosY_cor,"y--");
    plt::grid(); 

    plt::subplot(3,1,3);
    plt::Plot plot9_1("p_ref_z",posRef_Time_temp,posRef_Z_temp,"g");    
    plt::Plot plot9("PosDemandZ",posDemand_Time_temp,posDemand_Z_temp,"b");
    plt::Plot plot10("WritereadPosZ",canReadTime_Pos_temp,canReadData_PosZ_temp,"r");
    plt::grid();



    ///////////////////////////     global map     ///////////////////////////
    plt::figure(3);
    // plt::plot(PosY_cor,PosX_cor,{{"label", "f(x)"}});
    // sleep(5);
    plt::Plot plot_map("Global Map",PosY_cor,PosX_cor,"k"); 
    plt::Plot plot_map_leader("Leader",leader_posy_temp,leader_posx_temp,"xm"); 
    plt::Plot plot_map_planner("Planner",planner_posy_temp,planner_posx_temp,"g*");
    plt::title("Global Map");
    plt::grid();

    
    
    
  

    while(true){
        
        if(intp_button==false){
        
        /////////////////////////////////////////////////////////////////////////////
        ///////////////////////////     velocity part     //////////////////////////
        ///////////////////////////////////////////////////////////////////////////// 
        readFileToVector("../build/plot/CAN_Write_Time", canWriteTime_temp);
        readFileToVector("../build/plot/CAN_Write_Data_X", canWriteData_temp);
        readFileToVector("../build/plot/CAN_Write_Data_Y", canWriteData_Y_temp);
        readFileToVector("../build/plot/CAN_Write_Data_Z", canWriteData_Z_temp);
        

        readFileToVector("../build/plot/CAN_Read_Time", canReadTime_temp);
        readFileToVector("../build/plot/CAN_Read_Data_X", canReadData_temp);
        readFileToVector("../build/plot/CAN_Read_Data_Y", canReadData_Y_temp);
        readFileToVector("../build/plot/CAN_Read_Data_Z", canReadData_Z_temp);
        


        readFileToVector("../build/plot/VelDemand_Time", velDemand_Time_temp);
        readFileToVector("../build/plot/VelDemand_Data_X", velDemand_X_temp);
        readFileToVector("../build/plot/VelDemand_Data_Y", velDemand_Y_temp);
        readFileToVector("../build/plot/VelDemand_Data_Z", velDemand_Z_temp);

        readFileToVector("../build/plot/VelRef_Time", velRef_Time_temp);
        readFileToVector("../build/plot/VelRef_Data_X", velRef_X_temp);
        readFileToVector("../build/plot/VelRef_Data_Y", velRef_Y_temp);
        readFileToVector("../build/plot/VelRef_Data_Z", velRef_Z_temp);


        windowSizeControl();
           
        
        ///////////////////////////     velocity part  X   //////////////////////////
     
        if(canWriteTime_temp.size()==canWriteData_temp.size()){      
            
            plt::figure(1);
            plt::subplot(3,1,1);
            plt::xlim(0, wSize);
            plt::ylim(-280, 280);
            
            plot1.update(canWriteTime_temp,canWriteData_temp);

            if(canReadTime_temp.size()==canReadData_temp.size()){
               
                plot2.update(canReadTime_temp,canReadData_temp);
                
            }

            if(velDemand_Time_temp.size()==velDemand_X_temp.size()){
                
                plot0.update(velDemand_Time_temp,velDemand_X_temp);
                
            }

            if(velRef_Time_temp.size()==velRef_X_temp.size()){
                
                plot0_1.update(velRef_Time_temp,velRef_X_temp);
                
            }
        }
         plt::pause(0.05);

        ///////////////////////////     velocity part  Y   //////////////////////////

        if(canWriteTime_temp.size()==canWriteData_Y_temp.size()){      
          
          plt::figure(1);
          plt::subplot(3,1,2);
          plt::xlim(0, wSize);
          plt::ylim(-280, 280);
         
          plot3.update(canWriteTime_temp,canWriteData_Y_temp);
          std::cout<<"OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"<<std::endl;
          if(canReadTime_temp.size()==canReadData_Y_temp.size()){
                plot4.update(canReadTime_temp,canReadData_Y_temp);
            }

          if(velDemand_Time_temp.size()==velDemand_Y_temp.size()){
                
                plot2_9.update(velDemand_Time_temp,velDemand_Y_temp);
          }

          if(velRef_Time_temp.size()==velRef_Y_temp.size()){
                
                plot2_8.update(velRef_Time_temp,velRef_Y_temp);
                
            }

           else{
                std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
           }     
          
        }
        plt::pause(0.05);


        ///////////////////////////     velocity part  Z   //////////////////////////

        if(canWriteTime_temp.size()==canWriteData_Z_temp.size()){      
            
            plt::figure(1);
            plt::subplot(3,1,3);
            plt::xlim(0, wSize);
            plt::ylim(-20, 20);
            
            plot4_6.update(canWriteTime_temp,canWriteData_Z_temp);

            if(canReadTime_temp.size()==canReadData_Z_temp.size()){
               
                plot4_7.update(canReadTime_temp,canReadData_Z_temp);
                
            }

            if(velDemand_Time_temp.size()==velDemand_Z_temp.size()){
                
                plot4_5.update(velDemand_Time_temp,velDemand_Z_temp);
                
            }

            if(velRef_Time_temp.size()==velRef_Z_temp.size()){
                
                plot4_4.update(velRef_Time_temp,velRef_Z_temp);
                
            }
        }
         plt::pause(0.05); 


        

        /////////////////////////////////////////////////////////////////////////////
        ///////////////////////////     position part     ///////////////////////////
        /////////////////////////////////////////////////////////////////////////////

        readFileToVector("../build/plot/0_Leader_posx", leader_posx_temp);
        readFileToVector("../build/plot/0_Leader_posy", leader_posy_temp);

        readFileToVector("../build/plot/1_Planner_posx", planner_posx_temp);
        readFileToVector("../build/plot/1_Planner_posy", planner_posy_temp);


        readFileToVector("../build/plot/PosDemand_Time", posDemand_Time_temp);
        readFileToVector("../build/plot/PosDemand_Data_X", posDemand_X_temp);
        readFileToVector("../build/plot/PosDemand_Data_Y", posDemand_Y_temp);
        readFileToVector("../build/plot/PosDemand_Data_Z", posDemand_Z_temp);
        
        readFileToVector("../build/plot/CAN_Read_Time_Pos", canReadTime_Pos_temp);
        readFileToVector("../build/plot/CAN_Read_Data_PosX", canReadData_PosX_temp);
        readFileToVector("../build/plot/CAN_Read_Data_PosY", canReadData_PosY_temp);

        readFileToVector("../build/plot/CAN_Read_Data_PosX_Correct", PosX_cor);
        readFileToVector("../build/plot/CAN_Read_Data_PosY_Correct", PosY_cor);

        readFileToVector("../build/plot/CAN_Read_Time_PosZ", canReadTime_PosZ_temp);
        readFileToVector("../build/plot/CAN_Read_Data_PosZ", canReadData_PosZ_temp);

        readFileToVector("../build/plot/PosRef_Time", posRef_Time_temp);
        readFileToVector("../build/plot/PosRef_Data_X", posRef_X_temp);
        readFileToVector("../build/plot/PosRef_Data_Y", posRef_Y_temp);
        readFileToVector("../build/plot/PosRef_Data_Z", posRef_Z_temp);



        ///////////////////////////     position part X    ///////////////////////////
       
        if(posDemand_Time_temp.size()==posDemand_X_temp.size()){      
            plt::figure(2);
            plt::subplot(3,1,1);
            plt::xlim(0, wSize);
            plt::ylim(-300, 300);
            //plt::ylim(-100, 200);

            plot5.update(posDemand_Time_temp,posDemand_X_temp);

            if(canReadTime_Pos_temp.size()==canReadData_PosX_temp.size()){
                plot6.update(canReadTime_Pos_temp,canReadData_PosX_temp);
            }
            if(canReadTime_Pos_temp.size()==PosX_cor.size()){
                plot6_1.update(canReadTime_Pos_temp,PosX_cor);
            }
            else{
                std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            }

            if(posRef_Time_temp.size()==posRef_X_temp.size()){
                plot5_1.update(posRef_Time_temp,posRef_X_temp);
            }

            
       
        }
        

        //plt::pause(0.05);


        ///////////////////////////     position part Y    ///////////////////////////

        if(posDemand_Time_temp.size()==posDemand_Y_temp.size()){      
            //std::cout<<"OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"<<std::endl;
            plt::figure(2);
            plt::subplot(3,1,2);
            plt::xlim(0, wSize);
            plt::ylim(-300, 300);
            //plt::ylim(-100, 200);

            plot7.update(posDemand_Time_temp,posDemand_Y_temp);

            if(canReadTime_Pos_temp.size()==canReadData_PosY_temp.size()){
                plot8.update(canReadTime_Pos_temp,canReadData_PosY_temp);
            }
            if(canReadTime_Pos_temp.size()==PosY_cor.size()){
                plot8_1.update(canReadTime_Pos_temp,PosY_cor);
            }
            if(posRef_Time_temp.size()==posRef_Y_temp.size()){
                plot7_1.update(posRef_Time_temp,posRef_Y_temp);
            }
       
        }
        else{
            //std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }

        //plt::pause(0.05);

        ///////////////////////////     position part Z    ///////////////////////////

        if(posDemand_Time_temp.size()==posDemand_Z_temp.size()){      
            //std::cout<<"OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"<<std::endl;
            plt::figure(2);
            plt::subplot(3,1,3);
            plt::xlim(0, wSize);
            //plt::ylim(-150, 150);
            plt::ylim(-60, 60);

            plot9.update(posDemand_Time_temp,posDemand_Z_temp);

            if(canReadTime_PosZ_temp.size()==canReadData_PosZ_temp.size()){
                plot10.update(canReadTime_PosZ_temp,canReadData_PosZ_temp);
            }

            if(posRef_Time_temp.size()==posRef_Z_temp.size()){
                plot9_1.update(posRef_Time_temp,posRef_Z_temp);
            }

            else{
            std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            }
       
        }

        ///////////////////////////     Global Map    ///////////////////////////
        if(PosX_cor.size()==PosY_cor.size()){ 
            plt::figure(3);
            plt::xlim(-400, 400);
            plt::ylim(-400, 400);

            plot_map.update(PosY_cor,PosX_cor);

            if(leader_posx_temp.size()== leader_posy_temp.size()){

                plot_map_leader.update(leader_posy_temp,leader_posx_temp);
            }

            if(planner_posx_temp.size()== planner_posy_temp.size()){

                plot_map_planner.update(planner_posy_temp,planner_posx_temp);
            }

        }

        //plt::pause(0.05);


        leader_posx_temp.clear();
        leader_posy_temp.clear();

        planner_posx_temp.clear();
        planner_posy_temp.clear();



        //ref_vel
        velRef_Time_temp.clear();
        velRef_X_temp.clear();
        velRef_Y_temp.clear();
        velRef_Z_temp.clear();

        posRef_Time_temp.clear();
        posRef_X_temp.clear();
        posRef_Y_temp.clear();
        posRef_Z_temp.clear();

        
        

        //demand_vel
        velDemand_Time_temp.clear();
        velDemand_X_temp.clear();
        velDemand_Y_temp.clear();
        velDemand_Z_temp.clear();

        //demand_pos
        posDemand_Time_temp.clear();
        posDemand_X_temp.clear();
        posDemand_Y_temp.clear();
        posDemand_Z_temp.clear();



        canWriteTime_temp.clear();
        canReadTime_temp.clear();

        canWriteData_temp.clear();
        canReadData_temp.clear();
        
        canWriteData_Y_temp.clear();
        canReadData_Y_temp.clear();

        canWriteData_Z_temp.clear();
        canReadData_Z_temp.clear();


        canReadTime_Pos_temp.clear();
        canReadTime_PosZ_temp.clear();
        canReadData_PosX_temp.clear();
        canReadData_PosY_temp.clear();
        canReadData_PosZ_temp.clear();

        PosX_cor.clear();
        PosY_cor.clear();

        
        

        }//if(intp_button==false)

        if(show_angle_button){
            plotTheta();
            return;
        }
      
    }//while(true)

    

}


void plot(std::string CAN_Write_Time, std::string CAN_Write_Data, std::string CAN_Read_Time, std::string CAN_Read_Data){
    
   plt::Plot plot3("Writecan",canWriteTime_temp,canWriteData_temp,"k--");
    //plt::figure_size(1200, 780);
    //plt::grid();
    //plt::figure(2);
//plt::Plot plot;
    // plt::Plot plot1("Writecan",canWriteTime_Y_temp,canWriteData_temp,"k--");
    // plt::Plot plot2("Writeread",canReadTime_Y_temp,canReadData_temp,"r");

    // while(true){

    //     readFileToVector(CAN_Write_Time, canWriteTime_Y_temp);
    //     readFileToVector(CAN_Write_Data, canWriteData_Y_temp);

    //     readFileToVector(CAN_Read_Time, canReadTime_Y_temp);
    //     readFileToVector(CAN_Read_Data, canReadData_Y_temp);

    //     if(canWriteTime_Y_temp.size()==canWriteData_Y_temp.size()){
           
    //     //   plt::xlim(0, wSize);
    //     //   plt::ylim(-300, 300);

    //       plot.update(canWriteTime_Y_temp,canWriteData_Y_temp);

    //     //   if(canReadTime_Y_temp.size()==canReadData_Y_temp.size()){

    //     //         plot2.update(canReadTime_Y_temp,canReadData_Y_temp);
        
    //     //     }
    //     }
    
    //     plt::pause(0.05);
    //     canWriteTime_Y_temp.clear();
    //     canWriteData_Y_temp.clear();

    //     canReadData_Y_temp.clear();
    //     canReadTime_Y_temp.clear();
    //}

}

