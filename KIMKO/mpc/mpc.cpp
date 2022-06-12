#include "mpc.h"

MPC::MPC(int window){
window_planner = window;
pre_x_sol.resize(window_planner*3 + (window_planner-1)*6 + 2*(window_planner-1) + 1*window_planner,0);
initDemand();

}

//MPC


void MPC::mpcErrDyn_xy_plotPredicHorz(std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init, std::vector<std::vector<float>> Ori_ref, std::vector<float> Ori_init, std::vector<std::vector<float>> guess){
    // for dyn ref x direction
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    auto start = std::chrono::high_resolution_clock::now();
    
    pybind11::module_ mpc = pybind11::module_::import("errorDyn9_2_dynRef");
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess);
    
    std::vector<std::vector<float>> result_value = result.cast<std::vector<std::vector<float>>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    //std::cout << " MPC_DYN_errorDynOri9_2 operation time: "<< time2-time1 << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC_DYN_errorDynOri9_2 chrono time: " << float_ms.count() << " ms " << std::endl;
    mpcExTime = float_ms.count();

    x_pos_demand = result_value[0][0];
    y_pos_demand = result_value[0][1];
    
    float fsAngle_temp = result_value[0][2];
    fspeedVel_demand = result_value[0][3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand_rad = fsAngle_temp;
    fsAngle_demand = fsAngle_temp*180/PI;
    //std::cout << fsAngle_demand_rad << std::endl;


    float blank = result_value[0][4];
    float w_demand = result_value[0][5];

    z_pos_demand = result_value[0][6];
    z_vel_demand = result_value[0][7];

    predictHorz = result_value;
    
    for(int i=0;i<6;i++){
        pre_sol[i] = result_value[0][i];
    }
    

}

void MPC::mpcErrDyn_xy_plotPredicHorz_presol(std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init, std::vector<std::vector<float>> Ori_ref, std::vector<float> Ori_init, std::vector<std::vector<float>> guess){
    
    
    auto start = std::chrono::high_resolution_clock::now();
    

    //////////////////////////////////  Call Python Func and Extract Data   //////////////////////////////////
    pybind11::module_ mpc = pybind11::module_::import("errorDyn9_3_dynRef");
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess, pre_x_sol);
    
    //std::vector<std::vector<float>> result_value = result.cast<std::vector<std::vector<float>>>();
    std::vector<float> result_value = result.cast<std::vector<float>>();
    
    pre_x_sol=result_value;
    
    

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC_DYN_errorDynOri9_3 chrono time: " << float_ms.count() << " ms " << std::endl;
    mpcExTime = float_ms.count();

    


    //////////////////////////////////  Processing Predic Horz Data    //////////////////////////////////
    int window = window_planner;
    //std::cout << window << "\n";

    std::vector<float> pos_desired_temp;
    std::vector<float> ori_desired_temp;
    std::vector<std::vector<float>> predictHorz_temp;


    pos_desired_temp.resize(6*(window-1));
    ori_desired_temp.resize(2*(window-1));

    predictHorz_temp.resize(window-1);
    for (int i = 0; i < window-1; ++i){
        predictHorz_temp[i].resize(8);
    }
        
    int k=0;    

    for(int i = 3*window; i < 3*window+6*(window-1); i++) {
        pos_desired_temp[k] = result_value[i];
        k++;
    }

    k=0;
    for(int i = 3*window+6*(window-1); i < 3*window+6*(window-1)+2*(window-1); i++) {
        ori_desired_temp[k] = result_value[i];
        k++;
    }

    int it_pos=0;
    int it_ori=0;

    for(int w=0;w<(window-1);w++){
        std::vector<float> sub_pos = {pos_desired_temp.begin()+it_pos, pos_desired_temp.end()+it_pos+6};
        std::vector<float> sub_ori = {pos_desired_temp.begin()+it_ori, pos_desired_temp.end()+it_ori+2};
                
        predictHorz_temp[w].insert(predictHorz_temp[w].begin(), sub_pos.begin(), sub_pos.end());
        predictHorz_temp[w].insert(predictHorz_temp[w].end(), sub_ori.begin(), sub_ori.end());

        it_pos=it_pos+6;
        it_ori=it_ori+2;

    }

    predictHorz = predictHorz_temp;


    //////////////////////////////////  First element Predict Horz to Desired state    //////////////////////////////////

    x_pos_demand = pos_desired_temp[0];
    y_pos_demand = pos_desired_temp[1];
    
    float fsAngle_temp = pos_desired_temp[2];
    fspeedVel_demand = pos_desired_temp[3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand_rad = fsAngle_temp;
    fsAngle_demand = fsAngle_temp*180/PI;
    
    float blank = pos_desired_temp[4];
    float w_demand = pos_desired_temp[5];

    z_pos_demand = ori_desired_temp[0];
    z_vel_demand = ori_desired_temp[1];

}


void MPC::mpcObsAvoid_obsData(std::vector<std::vector<int>> obs,std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init, std::vector<std::vector<float>> Ori_ref, std::vector<float> Ori_init, std::vector<std::vector<float>> guess){
    
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    auto start = std::chrono::high_resolution_clock::now();
    
    pybind11::module_ mpc = pybind11::module_::import("obsAvoid_3");
    pybind11::object result = mpc.attr("errDynFunction")(obs, p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess);
    std::vector<std::vector<float>> result_value = result.cast<std::vector<std::vector<float>>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    // std::cout << "MPC ObsAvoid obsData time: "<< time2-time1 << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC ObsAvoid obsData time: " << float_ms.count() << " ms " << std::endl;
    mpcExTime = float_ms.count();

    x_pos_demand = result_value[0][0];
    y_pos_demand = result_value[0][1];
    
    float fsAngle_temp = result_value[0][2];
    fspeedVel_demand = result_value[0][3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand_rad = fsAngle_temp;
    fsAngle_demand = fsAngle_temp*180/PI;
    //std::cout << fsAngle_demand_rad << std::endl;


    float blank = result_value[0][4];
    float w_demand = result_value[0][5];

    z_pos_demand = result_value[0][6];
    z_vel_demand = result_value[0][7];

    predictHorz = result_value;
    
    for(int i=0;i<6;i++){
        pre_sol[i] = result_value[0][i];
    }
    

}

void MPC::mpcAvoid_obsData_presol(std::vector<std::vector<int>> obs,std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init, std::vector<std::vector<float>> Ori_ref, std::vector<float> Ori_init, std::vector<std::vector<float>> guess){
    
    
    auto start = std::chrono::high_resolution_clock::now();
    

    pybind11::module_ mpc = pybind11::module_::import("obsAvoid_4_presol");
    
    //////////////////////////////////  Call Python Func and Extract Data   //////////////////////////////////

    pybind11::object result = mpc.attr("errDynFunction")(obs, p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess, pre_x_sol);
    //std::vector<std::vector<float>> result_value = result.cast<std::vector<std::vector<float>>>();
    std::vector<float> result_value = result.cast<std::vector<float>>();
    
    pre_x_sol=result_value;
  
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC ObsAvoid obsData time: " << float_ms.count() << " ms " << std::endl;
    mpcExTime = float_ms.count();


    //////////////////////////////////  Processing Predic Horz Data    //////////////////////////////////
    int window = window_planner;
    //std::cout << window << "\n";

    std::vector<float> pos_desired_temp;
    std::vector<float> ori_desired_temp;
    std::vector<std::vector<float>> predictHorz_temp;


    pos_desired_temp.resize(6*(window-1));
    ori_desired_temp.resize(2*(window-1));

    predictHorz_temp.resize(window-1);
    for (int i = 0; i < window-1; ++i){
        predictHorz_temp[i].resize(8);
    }
    
    int k=0;
 
    for(int i = 3*window; i < 3*window+6*(window-1); i++) {
        pos_desired_temp[k] = result_value[i];
        k++;
    }

    k=0;
    for(int i = 3*window+6*(window-1); i < 3*window+6*(window-1)+2*(window-1); i++) {
        ori_desired_temp[k] = result_value[i];
        k++;
    }

    int it_pos=0;
    int it_ori=0;
    for(int w=0;w<(window-1);w++){
        std::vector<float> sub_pos = {pos_desired_temp.begin()+it_pos, pos_desired_temp.end()+it_pos+6};
        std::vector<float> sub_ori = {pos_desired_temp.begin()+it_ori, pos_desired_temp.end()+it_ori+2};
                
        predictHorz_temp[w].insert(predictHorz_temp[w].begin(), sub_pos.begin(), sub_pos.end());
        predictHorz_temp[w].insert(predictHorz_temp[w].end(), sub_ori.begin(), sub_ori.end());

        it_pos=it_pos+6;
        it_ori=it_ori+2;
    }

    predictHorz = predictHorz_temp;

    //////////////////////////////////  First element Predict Horz to Desired state    //////////////////////////////////

    x_pos_demand = pos_desired_temp[0];
    y_pos_demand = pos_desired_temp[1];
    
    float fsAngle_temp = pos_desired_temp[2];
    fspeedVel_demand = pos_desired_temp[3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand_rad = fsAngle_temp;
    fsAngle_demand = fsAngle_temp*180/PI;
    
    float blank = pos_desired_temp[4];
    float w_demand = pos_desired_temp[5];

    z_pos_demand = ori_desired_temp[0];
    z_vel_demand = ori_desired_temp[1];

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

    

    else if(time>=6){
        velDemand=0;
    }

    else{
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
