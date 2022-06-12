#include <fstream>
#include <vector>
#include <iostream>
#include <stdlib.h>

#include "../matplotlib-cpp-master/matplotlibcpp.h"

namespace plt = matplotlibcpp;


std::vector<float> obs_posx_temp;
std::vector<float> obs_posy_temp;

std::vector<float> predictHorz_posx_temp;
std::vector<float> predictHorz_posy_temp;

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

std::vector<float> fspeedRef_temp;
std::vector<float> fsAngleRef_temp;


std::vector<float> posDemand_Time_temp;
std::vector<float> posDemand_X_temp;
std::vector<float> posDemand_Y_temp;
std::vector<float> posDemand_Z_temp;

std::vector<float> velDemand_Time_temp;
std::vector<float> velDemand_X_temp;
std::vector<float> velDemand_Y_temp;
std::vector<float> velDemand_Z_temp;

std::vector<float> fspeedDemand_temp;
std::vector<float> fsAngleDemand_temp;

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

std::vector<float> canReadData_PosZ_temp;

std::vector<float> canReadData_PosX_temp;
std::vector<float> canReadData_PosY_temp;

std::vector<float> PosX_cor;
std::vector<float> PosY_cor;

std::vector<float> fspeedRobot_temp;
std::vector<float> fsAngleRobot_temp;


int wSize = 20;

bool intp_button = false;
bool show_angle_button = false;

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


void cleanVector(){
    obs_posx_temp.clear();
        obs_posy_temp.clear();

        leader_posx_temp.clear();
        leader_posy_temp.clear();

        planner_posx_temp.clear();
        planner_posy_temp.clear();

        predictHorz_posx_temp.clear();
        predictHorz_posy_temp.clear();

        //ref_vel
        velRef_Time_temp.clear();
        velRef_X_temp.clear();
        velRef_Y_temp.clear();
        velRef_Z_temp.clear();

        fspeedRef_temp.clear();
        fsAngleRef_temp.clear();

        posRef_Time_temp.clear();
        posRef_X_temp.clear();
        posRef_Y_temp.clear();
        posRef_Z_temp.clear();
 
        //demand_vel
        velDemand_Time_temp.clear();
        velDemand_X_temp.clear();
        velDemand_Y_temp.clear();
        velDemand_Z_temp.clear();

        fspeedDemand_temp.clear();
        fsAngleDemand_temp.clear();

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

        fspeedRobot_temp.clear();
        fsAngleRobot_temp.clear();

}


