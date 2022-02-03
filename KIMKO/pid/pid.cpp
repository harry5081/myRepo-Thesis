#include <iostream>
#include <vector>
#include <unistd.h>
#include "pid.h"



#include "../plot/readWritePlot.h"

// MPC
std::vector<float> selfDefPosDemand;
std::vector<float> selfDefVelDemand;

float amplitude = 100;
float period = 10;






// plot PositionDemand_X
ReadWritePlot *PosDemandDataPlot_X = new ReadWritePlot;
ReadWritePlot *PosDemandTimePlot_X = new ReadWritePlot;

// plot velocityDemand_X
ReadWritePlot *VelDemandDataPlot_X = new ReadWritePlot;
ReadWritePlot *VelDemandTimePlot_X = new ReadWritePlot;


//PID

PID::PID(float Kp_set, float Ki_set, float Kf_set, float Kd_set):
kp(Kp_set),
ki(Ki_set),
kf(Kf_set),
kd(Kd_set)
{
    std::cout<< " PID Init" <<std::endl;

}


int PID::pidExe(float posError, int velDemand, float velError){
   

    //std::cout<< "pidExe_KpKiKfKd_Pos+Vel" <<std::endl;

    float time = (float)clock()/CLOCKS_PER_SEC;

    /////////////////////////////////////////////////////////////////////////////
    ///////////////////////////     position part     ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    //posDemand=powThreePosDemand(time);
    //posDemand=sinePosDemand(time);
    //posDemand=sineToTenPosDemand(time);
    
    //PosDemandTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/PosDemand_Time");
    //PosDemandDataPlot_X->writeDatatoFile(posDemand, "plot/PosDemand_Data_X");
    

    /////////////////////////////////////////////////////////////////////////////
    ///////////////////////////     velocity part     ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    //velDemand = powTwoVelDemand(time);
    //velDemand = cosVelDemand(time);
    //velDemand = cosToTenVelDemand(time);


    //VelDemandTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/VelDemand_Time");
    //VelDemandDataPlot_X->writeDatatoFile(velDemand, "plot/VelDemand_Data_X");

    //posError=posDemand-posError;
    //velError=velDemand-velError;

    //vRef = error * Kp + errorAccumulate * Ki;
    //vRef = error * Kp + errorAccumulate * Ki + (velDemand-velSenValue)*Kd;
    vInput = posError * kp + errorAccumulate * ki + velDemand*kf + velError*kd;




    //usleep(10000);


    // if(vRef > 500){

    //     vRef =100;
    //     std::cout<<" XY Speed too fast!!"<<std::endl;

    // }

    // else if(vRef < -500){

    //     vRef =-100;
    //     std::cout<<" XY Speed too fast!!"<<std::endl;


    // }

    errorAccumulate = errorAccumulate + posError;
    // std::cout<< "Position Error: " << posError <<std::endl;
    // std::cout<< std::endl;
    // std::cout<< std::endl;
    

    return vInput;

}

int PID::pidExeAngle(float posError, int velDemand, float velError){
   

    //std::cout<< "pidExe_KpKiKfKd_Angle" <<std::endl;

    float time = (float)clock()/CLOCKS_PER_SEC;

    /////////////////////////////////////////////////////////////////////////////
    ///////////////////////////     position part     ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    //posDemand=powThreePosDemand(time);
    //posDemand=sinePosDemand(time);
    //posDemand=sineToTenPosDemand(time);
    
    //PosDemandTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/PosDemand_Time");
    //PosDemandDataPlot_X->writeDatatoFile(posDemand, "plot/PosDemand_Data_X");
    

    /////////////////////////////////////////////////////////////////////////////
    ///////////////////////////     velocity part     ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    //velDemand = powTwoVelDemand(time);
    //velDemand = cosVelDemand(time);
    //velDemand = cosToTenVelDemand(time);


    //VelDemandTimePlot_X->writeDatatoFile((float)clock()/CLOCKS_PER_SEC, "plot/VelDemand_Time");
    //VelDemandDataPlot_X->writeDatatoFile(velDemand, "plot/VelDemand_Data_X");

    //posError=posDemand-posError;
    //velError=velDemand-velError;

    //vRef = error * Kp + errorAccumulate * Ki;
    //vRef = error * Kp + errorAccumulate * Ki + (velDemand-velSenValue)*Kd;
    vInput = posError * kp + errorAccumulate * ki + velDemand*kf + velError*kd;




    //usleep(10000);


    // if(vRef > 50){

    //     vRef =10;
    //     std::cout<<" Angle Speed too fast!!"<<std::endl;

    // }

    // else if(vRef < -50){

    //     vRef =-10;
    //     std::cout<<" Angle Speed too fast!!"<<std::endl;


    // }

    errorAccumulate = errorAccumulate + posError;
    // std::cout<< "Position Error: " << posError <<std::endl;
    // std::cout<< std::endl;
    // std::cout<< std::endl;
    

    return vInput;

}