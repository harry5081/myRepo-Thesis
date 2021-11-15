#include "mpc.h"

MPC::MPC(){

initDemand();

}

//MPC
void MPC::initDemand(){ //initialize sine positionDemand

    std::cout<< "MPC demand init." <<std::endl;

    for(int i=0; i<100; i++){
        
        selfDefPosDemand.push_back(amplitude*sin(float(i)/100*(2*PI)));
    }

    for(int i=0; i<100; i++){
        
        selfDefVelDemand.push_back(amplitude*cos(float(i)/100*(2*PI)));
    }

    /*for(auto i:selfDefPosDemand)
    {

        std::cout<< i << " ";
    }
    
    std::cout<<std::endl;
    */

}





float MPC::sinePosDemand(float time){

    float posDemand;

    if(time<=period){
        posDemand=selfDefPosDemand[int((time)*(100/period))];
    }

    else {
       posDemand=selfDefPosDemand[int((time-((int(time))-((int(time))%int(period))))*100/period)];
        
    }

    return posDemand;
}

float MPC::cosVelDemand(float time){

    

    float velDemand;

    if(time<=period){
        velDemand=selfDefVelDemand[int((time)*(100/period))];
    }

    else {
       velDemand=selfDefVelDemand[int((time-((int(time))-((int(time))%int(period))))*100/period)];
        
    }

    return velDemand;
}



float MPC::powThreePosDemand(float time){

    float posDemand;

    if(time<=3.0){
        posDemand=0;
    }

    else if(time>8){
       posDemand=150;
  
    }

    else{
        posDemand = (15) * ( (-4)*pow(time-3, 3)/25 + 6*pow(time-3, 2)/5);

    }

    return posDemand;
}

float MPC::powTwoVelDemand(float time){

    float velDemand;

    if(time<=3){
        velDemand=0;
    }

    else if(time>8){
       velDemand=0;
  
    }

    else{
        velDemand = (15) * ( (-12)*pow(time-3, 2)/25 + 12*(time-3)/5);

    }

    return velDemand;
}

float MPC::stepPosDemand(float time){

    float posDemand;
    
    if(time<=10){
        posDemand=0;
    }

    else {
       posDemand=100;
        
    }

    return posDemand;
}


float MPC::sineToTenPosDemand(float time){

    float posDemand;

    if(time<=2*PI-3){
        posDemand=0;
    }

    else if(time>2*PI-3+PI){
       posDemand=100;
  
    }

    else{
        posDemand = (50) * (sin(time+2*PI*3/4+3)+1);

    }

    return posDemand;
}

float MPC::cosToTenVelDemand(float time){

    float velDemand;

    if(time<=2*PI-3){
        velDemand=0;
    }

    else if(time>2*PI-3+PI){
       velDemand=0;
  
    }

    else{
        velDemand = (50.0) * ( cos(time+2*PI*3/4+3));

    }

    return velDemand;
}
