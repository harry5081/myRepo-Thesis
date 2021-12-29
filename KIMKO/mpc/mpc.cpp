#include "mpc.h"

MPC::MPC(){

initDemand();

}

//MPC




void MPC::mpcOperation(float v_ref, float p_ref, float v_init, float p_init, int v_input_begin){
    
    float time1 = (float)clock()/CLOCKS_PER_SEC;

    pybind11::module_ mpc = pybind11::module_::import("mpc_xDirect_multi_test");
    //pybind11::object result = mpc.attr("functionTest")(v_ref, p_ref, v_init, p_init, v_input_begin);//(v_ref, p_ref, v_init, p_init, v_input_begin);
    
    pybind11::object result = mpc.attr("functionTest2")(v_ref, p_ref, v_init, p_init, v_input_begin,pre_vd,pre_pd);//(v_ref, p_ref, v_init, p_init, v_input_begin);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC operation time: "<< time2-time1 << std::endl;

    x_vel_demand = result_value[0];
    x_pos_demand = result_value[1];

    pre_vd = x_vel_demand;
    pre_pd = x_pos_demand;

    //auto result_value= result.cast;
    //std::cout << x_vel_demand << std::endl;
    //std::cout << x_pos_demand << std::endl;

    

}

void MPC::mpcOperation(std::vector<float> v_ref, std::vector<float> p_ref, std::vector<float> v_init, std::vector<float> p_init, std::vector<float> v_input){

    float time1 = (float)clock()/CLOCKS_PER_SEC;
    
    pybind11::module_ mpc = pybind11::module_::import("mpc_xyzDirect");
    pybind11::object result = mpc.attr("functionTest")(v_ref, p_ref, v_init, p_init, v_input, pre_vd_pd);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC_XYZ operation time: "<< time2-time1 << std::endl;

    x_vel_demand = result_value[0];
    x_pos_demand = result_value[1];

    y_vel_demand = result_value[2];
    y_pos_demand = result_value[3];

    z_vel_demand = result_value[4];
    z_pos_demand = result_value[5];

    pre_vd_pd = {x_vel_demand, x_pos_demand, y_vel_demand, y_pos_demand, z_vel_demand, z_pos_demand};


    std::cout << z_vel_demand << std::endl;
    std::cout << z_pos_demand << std::endl;
}


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

float MPC::stepVelDemand(float time){

    float velDemand;
    
    if(time<=3){
        velDemand=0;
    }

    else {
       velDemand=100;
        
    }

    return  velDemand;
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
