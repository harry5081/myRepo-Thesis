#include "mpc.h"

MPC::MPC(int window){
window_planner = window;
pre_x_sol.resize(window_planner*3 + (window_planner-1)*6 + 2*(window_planner-1) + 1*window_planner,0);
initDemand();

}

//MPC

void MPC::mpcErrDyn_xy_ori(std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init, std::vector<std::vector<float>> Ori_ref, std::vector<float> Ori_init, std::vector<std::vector<float>> guess){
    // for dyn ref x direction
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    auto start = std::chrono::high_resolution_clock::now();
    
    pybind11::module_ mpc = pybind11::module_::import("errorDyn9_2_dynRef");
    //pybind11::module_ mpc = pybind11::module_::import("errorDyn6_dynRef_xy");
    //pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init, pre_sol);
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC_DYN_errorDynOri8_3 operation time: "<< time2-time1 << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC_DYN_errorDynOri8_3 chrono time: " << float_ms.count() << " ms " << std::endl;
    mpcExTime = float_ms.count();

    x_pos_demand = result_value[0];
    y_pos_demand = result_value[1];
    
    float fsAngle_temp = result_value[2];
    fspeedVel_demand = result_value[3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand_rad = fsAngle_temp;
    fsAngle_demand = fsAngle_temp*180/PI;
    //std::cout << fsAngle_demand_rad << std::endl;


    float blank = result_value[4];
    float w_demand = result_value[5];

    z_pos_demand = result_value[6];
    z_vel_demand = result_value[7];
    
    for(int i=0;i<6;i++){
        pre_sol[i] = result_value[i];
    }
    

}

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
    
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    auto start = std::chrono::high_resolution_clock::now();
    
    pybind11::module_ mpc = pybind11::module_::import("errorDyn9_3_dynRef");
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess, pre_x_sol);
    //std::vector<std::vector<float>> result_value = result.cast<std::vector<std::vector<float>>>();
    std::vector<float> result_value = result.cast<std::vector<float>>();
    pre_x_sol=result_value;
    float time2 = (float)clock()/CLOCKS_PER_SEC;
    // std::cout << "MPC ObsAvoid obsData time: "<< time2-time1 << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC_DYN_errorDynOri9_3 chrono time: " << float_ms.count() << " ms " << std::endl;
    mpcExTime = float_ms.count();

    //x_pos_demand = result_value[0];
    // y_pos_demand = result_value[0][1];
    
    // float fsAngle_temp = result_value[0][2];
    // fspeedVel_demand = result_value[0][3];

    // //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    // //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    // fsAngle_demand_rad = fsAngle_temp;
    // fsAngle_demand = fsAngle_temp*180/PI;
    // //std::cout << fsAngle_demand_rad << std::endl;


    // float blank = result_value[0][4];
    // float w_demand = result_value[0][5];

    // z_pos_demand = result_value[0][6];
    // z_vel_demand = result_value[0][7];

    // predictHorz = result_value;
    
    // for(int i=0;i<6;i++){
    //     pre_sol[i] = result_value[0][i];
    // }

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
    //std::cout << result_value[i] << "\n";
    pos_desired_temp[k] = result_value[i];
    k++;
    }

    k=0;
    for(int i = 3*window+6*(window-1); i < 3*window+6*(window-1)+2*(window-1); i++) {
    //std::cout << *it << "\n";
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

    // for(auto it = std::begin(result_value)+3*10; it != std::begin(result_value)+3*10+6*9; ++it) {
    // //std::cout << *it << "\n";
    // pos_desired_temp[i] = *it;
    // i++;
    // }

    // for(auto it = std::begin(result_value)+3*10+6*9; it != std::begin(result_value)+3*10+6*9+2*9; ++it) {
    // //std::cout << *it << "\n";
    // ori_desired_temp[j] = *it;
    // j++;
    // }

    // for(int it){


    // }

    x_pos_demand = pos_desired_temp[0];
    y_pos_demand = pos_desired_temp[1];
    
    float fsAngle_temp = pos_desired_temp[2];
    fspeedVel_demand = pos_desired_temp[3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand_rad = fsAngle_temp;
    fsAngle_demand = fsAngle_temp*180/PI;
    //std::cout << fsAngle_demand_rad << std::endl;


    float blank = pos_desired_temp[4];
    float w_demand = pos_desired_temp[5];

    z_pos_demand = ori_desired_temp[0];
    z_vel_demand = ori_desired_temp[1];

}

void MPC::mpcObsAvoid(std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init, std::vector<std::vector<float>> Ori_ref, std::vector<float> Ori_init, std::vector<std::vector<float>> guess){
    
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    auto start = std::chrono::high_resolution_clock::now();
    
    pybind11::module_ mpc = pybind11::module_::import("obsAvoid");
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess);
    std::vector<std::vector<float>> result_value = result.cast<std::vector<std::vector<float>>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << "MPC ObsAvoid operation time: "<< time2-time1 << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC ObsAvoid chrono time: " << float_ms.count() << " ms " << std::endl;
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
    
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    auto start = std::chrono::high_resolution_clock::now();
    
    pybind11::module_ mpc = pybind11::module_::import("obsAvoid_4_presol");
    pybind11::object result = mpc.attr("errDynFunction")(obs, p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess, pre_x_sol);
    //std::vector<std::vector<float>> result_value = result.cast<std::vector<std::vector<float>>>();
    std::vector<float> result_value = result.cast<std::vector<float>>();
    pre_x_sol=result_value;
    float time2 = (float)clock()/CLOCKS_PER_SEC;
    // std::cout << "MPC ObsAvoid obsData time: "<< time2-time1 << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;
    std::cout << "MPC ObsAvoid obsData time: " << float_ms.count() << " ms " << std::endl;
    mpcExTime = float_ms.count();

    //x_pos_demand = result_value[0];
    // y_pos_demand = result_value[0][1];
    
    // float fsAngle_temp = result_value[0][2];
    // fspeedVel_demand = result_value[0][3];

    // //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    // //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    // fsAngle_demand_rad = fsAngle_temp;
    // fsAngle_demand = fsAngle_temp*180/PI;
    // //std::cout << fsAngle_demand_rad << std::endl;


    // float blank = result_value[0][4];
    // float w_demand = result_value[0][5];

    // z_pos_demand = result_value[0][6];
    // z_vel_demand = result_value[0][7];

    // predictHorz = result_value;
    
    // for(int i=0;i<6;i++){
    //     pre_sol[i] = result_value[0][i];
    // }

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
    //std::cout << result_value[i] << "\n";
    pos_desired_temp[k] = result_value[i];
    k++;
    }

    k=0;
    for(int i = 3*window+6*(window-1); i < 3*window+6*(window-1)+2*(window-1); i++) {
    //std::cout << *it << "\n";
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

    // for(auto it = std::begin(result_value)+3*10; it != std::begin(result_value)+3*10+6*9; ++it) {
    // //std::cout << *it << "\n";
    // pos_desired_temp[i] = *it;
    // i++;
    // }

    // for(auto it = std::begin(result_value)+3*10+6*9; it != std::begin(result_value)+3*10+6*9+2*9; ++it) {
    // //std::cout << *it << "\n";
    // ori_desired_temp[j] = *it;
    // j++;
    // }

    // for(int it){


    // }

    x_pos_demand = pos_desired_temp[0];
    y_pos_demand = pos_desired_temp[1];
    
    float fsAngle_temp = pos_desired_temp[2];
    fspeedVel_demand = pos_desired_temp[3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand_rad = fsAngle_temp;
    fsAngle_demand = fsAngle_temp*180/PI;
    //std::cout << fsAngle_demand_rad << std::endl;


    float blank = pos_desired_temp[4];
    float w_demand = pos_desired_temp[5];

    z_pos_demand = ori_desired_temp[0];
    z_vel_demand = ori_desired_temp[1];

}


void MPC::mpcErrDyn_xy(std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init){
    // for dyn ref x direction
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    auto start = std::chrono::high_resolution_clock::now();
    
    pybind11::module_ mpc = pybind11::module_::import("errorDyn7_6_3_dynRef");
    //pybind11::module_ mpc = pybind11::module_::import("errorDyn6_dynRef_xy");
    //pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init, pre_sol);
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC_DYN_errorDynXY operation time: "<< time2-time1 << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> float_ms = end - start;

    std::cout << "MPC_DYN_errorDynXY chrono time: " << float_ms.count() << " milliseconds" << std::endl;



    x_pos_demand = result_value[0];
    y_pos_demand = result_value[1];
    
    float fsAngle_temp = result_value[2];
    fspeedVel_demand = result_value[3];

    //x_vel_demand=fspeedVel_demand*cos(fsAngle_temp);
    //y_vel_demand=fspeedVel_demand*sin(fsAngle_temp);

    fsAngle_demand = fsAngle_temp*180/PI;


    float blank = result_value[4];
    float w_demand = result_value[5];

    pre_sol = result_value;

}

void MPC::mpcErrDyn(std::vector<float> p_ref, std::vector<float> v_ref, std::vector<float> p_init, std::vector<float> v_init){
    // for fix ref y direction
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    
    pybind11::module_ mpc = pybind11::module_::import("errorDyn5_fixRef_forY");
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC_Y_errorDyn1 operation time: "<< time2-time1 << std::endl;

    // x_pos_demand = result_value[0];
    // y_pos_demand = result_value[1];
    // float phi_demand = result_value[2];


    // float v_demand = result_value[3];
    // x_vel_demand=v_demand;
    // float blank = result_value[4];
    // float w_demand = result_value[5];

    fsAngle_demand = result_value[2]*180/PI;
    fspeedVel_demand = result_value[3];

    x_pos_demand = result_value[0];
    y_pos_demand = result_value[1];
    float phi_demand = result_value[2];

    
    float v_demand = result_value[3];
    y_vel_demand=v_demand*sin(phi_demand);

    y_vel_ref = fspeedVel_ref; // for plot

    float blank = result_value[4];
    float w_demand = result_value[5];

    std::cout << "fspeed: "<< v_demand << std::endl;
    std::cout << "angle: "<< sin(phi_demand)*180/PI << std::endl;
    std::cout << y_vel_demand << std::endl;

    //pre_vd_pd = {x_vel_demand, x_pos_demand, y_vel_demand, y_pos_demand, z_vel_demand, z_pos_demand};


    //std::cout << z_vel_demand << std::endl;
    //std::cout << z_pos_demand << std::endl;
}   

void MPC::mpcErrDyn(std::vector<std::vector<float>> p_ref, std::vector<std::vector<float>> v_ref, std::vector<float> p_init, std::vector<float> v_init){
    // for dyn ref x direction
    float time1 = (float)clock()/CLOCKS_PER_SEC;
    
    pybind11::module_ mpc = pybind11::module_::import("errorDyn5_dynRef");
    pybind11::object result = mpc.attr("errDynFunction")(p_ref, v_ref, p_init, v_init);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC_DYN_errorDyn operation time: "<< time2-time1 << std::endl;

    x_pos_demand = result_value[0];
    y_pos_demand = result_value[1];
    float phi_demand = result_value[2];


    float v_demand = result_value[3];
    x_vel_demand=v_demand;
    float blank = result_value[4];
    float w_demand = result_value[5];

}
    


void MPC::mpcOperation(float v_ref, float p_ref, float v_init, float p_init, int v_input_begin){
    
    float time1 = (float)clock()/CLOCKS_PER_SEC;

    pybind11::module_ mpc = pybind11::module_::import("mpc_xDirect_multi_test");
    //pybind11::object result = mpc.attr("functionTest")(v_ref, p_ref, v_init, p_init, v_input_begin);//(v_ref, p_ref, v_init, p_init, v_input_begin);
    
    pybind11::object result = mpc.attr("functionTest2")(v_ref, p_ref, v_init, p_init, v_input_begin, pre_vd, pre_pd);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC operation time: "<< time2-time1 << std::endl;

    x_vel_demand = result_value[0];
    x_pos_demand = result_value[1];

    // z_vel_demand = result_value[0];
    // z_pos_demand = result_value[1];

    pre_vd = x_vel_demand;
    pre_pd = x_pos_demand;

    //auto result_value= result.cast;
    //std::cout << x_vel_demand << std::endl;
    //std::cout << x_pos_demand << std::endl;

    

}

void MPC::mpcOperation(std::vector<float> v_ref, std::vector<float> p_ref, std::vector<float> v_init, std::vector<float> p_init, std::vector<float> v_input){

    float time1 = (float)clock()/CLOCKS_PER_SEC;
    
    pybind11::module_ mpc = pybind11::module_::import("mpc_xyzDirect_couple");
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


    //std::cout << z_vel_demand << std::endl;
    //std::cout << z_pos_demand << std::endl;
}


void MPC::mpcOperation(std::vector<std::vector<float>> v_ref, std::vector<std::vector<float>> p_ref, std::vector<float> v_init, std::vector<float> p_init, std::vector<float> v_input){

    float time1 = (float)clock()/CLOCKS_PER_SEC;
    
    //pybind11::module_ mpc = pybind11::module_::import("tra_mpc_xyzDirect_exp_couple");
    pybind11::module_ mpc = pybind11::module_::import("noexp_tra_mpc_xyzDirect_exp_couple");
    pybind11::object result = mpc.attr("functionTest")(v_ref, p_ref, v_init, p_init, v_input, pre_vd_pd);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    

    x_vel_demand = result_value[0];
    x_pos_demand = result_value[1];

    y_vel_demand = result_value[2];
    y_pos_demand = result_value[3];

    z_vel_demand = result_value[4];
    z_pos_demand = result_value[5];

    

    pre_vd_pd = {x_vel_demand, x_pos_demand, y_vel_demand, y_pos_demand, z_vel_demand, z_pos_demand};

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC_dyn operation time: "<< time2-time1 << std::endl;


    //std::cout << z_vel_demand << std::endl;
    //std::cout << z_pos_demand << std::endl;
}

void MPC::mpcOperation(std::vector<std::vector<float>> v_ref, std::vector<std::vector<float>> p_ref, std::vector<float> v_init, std::vector<float> p_init, std::vector<float> v_input,std::vector<std::vector<float>>fspeed_ref, std::vector<float> fspeed_init){

    float time1 = (float)clock()/CLOCKS_PER_SEC;
    
    //pybind11::module_ mpc = pybind11::module_::import("fspeed_tra_mpc_xyzDirect_exp_couple");
    pybind11::module_ mpc = pybind11::module_::import("fspeed_noexp_tra_mpc_xyzDirect_exp_couple");
    pybind11::object result = mpc.attr("functionTest")(v_ref, p_ref, v_init, p_init, v_input, pre_vd_pd, fspeed_ref, fspeed_init);
    std::vector<float> result_value = result.cast<std::vector<float>>();

    

    x_vel_demand = result_value[0];
    x_pos_demand = result_value[1];

    y_vel_demand = result_value[2];
    y_pos_demand = result_value[3];

    z_vel_demand = result_value[4];
    z_pos_demand = result_value[5];

    

    pre_vd_pd = {x_vel_demand, x_pos_demand, y_vel_demand, y_pos_demand, z_vel_demand, z_pos_demand};

    float time2 = (float)clock()/CLOCKS_PER_SEC;
    std::cout << " MPC_dyn_fspeed operation time: "<< time2-time1 << std::endl;


    //std::cout << z_vel_demand << std::endl;
    //std::cout << z_pos_demand << std::endl;
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
