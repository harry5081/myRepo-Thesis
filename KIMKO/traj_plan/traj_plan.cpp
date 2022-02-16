#include "traj_plan.h"
#include <vector>

PLANNER::PLANNER(){
    
    r = leaderData.r;

    std::vector<float> temp={0,0,0};
    
     for(int w =0;w<window;w++)
     {

        pos_ref.push_back(temp);
        vel_ref.push_back(temp);

        fspeed_ref.push_back(temp);

     }


}

void PLANNER::cir_traject(){
    
    std::cout << s <<std::endl;
    dt = (s-pre_s)/r/window;

    t_current= t+dt;
    
    for(int i =0;i<window;i++){

        t=t+dt;
       

        float xt = r*sin(t);
        float yt = r*-cos(t)+r;

        std::vector<float> point = {xt,yt,0};
        pos_ref[i] = point;






        //pos_ref.push_back(point);
        //std::cout << xt    << " "<< yt <<std::endl;
               
         
        //usleep(100000);

    }

    t=t_current;

    
    pre_s = t_current*r;



}

void PLANNER::cir_traject_2(){
    
    std::cout << s <<std::endl;
    dt = sampleTime;

    t=t_current;
    fsAngle_pre_window = fsAngle_pre; /********* angle unwrap *********/

    for(int i =0;i<window;i++){

        if((t+dt)<=s/r/w){
            t=t+dt;

        }
        /////////////////////////////////////  pos  ////////////////////////////////////////
        float xt = r*sin(w*t);
        float yt = r*-cos(w*t)+r;
        float zt =20;

        


        /////////////////////////////////////  vel  ////////////////////////////////////////
        float vt_x = r*w*cos(w*t);
        float vt_y = r*w*sin(w*t);
        //float vt = sqrt(pow(vt_x,2)+pow(vt_y,2));

        float vn_x = -r*w*sin(w*t);
        float vn_y = r*w*cos(w*t);
        float vn = sqrt(pow(vn_x,2)+pow(vn_y,2));


        /////////////////////////////////////  forward speed  ////////////////////////////////////////
        float fspeed = sqrt(pow(vt_x,2)+pow(vt_y,2));
        float fsAngle = atan2(vt_x,vt_y)*180/PI;
        
        
        /********* angle unwrap *********/
        float fsAngle_unwrap = unwrap(fsAngle_pre_window,fsAngle);
        fsAngle_pre_window=fsAngle_unwrap;

        if(i==0){

            fsAngle_pre = fsAngle_pre_window;
        }
        /********* angle unwrap *********/

        
        
        float vx=vt_x;//vt_x;//vt_x;
        float vy=vt_y;//vt_y;

        if((t+dt)>=s/r/w){
            vx=0;
            vy=0;

            fspeed=0;
            fsAngle=0;
            fsAngle_unwrap=0;

            zt =0;
        }



        std::vector<float> point = {xt,0,0};
        pos_ref[i] = point;


        std::vector<float> vel = {vx,0,0};
        // std::vector<float> vel = {0,0,0};
        vel_ref[i] = vel;

        std::vector<float> fspeed_temp = {fspeed,fsAngle};
        //std::vector<float> fspeed_temp = {fspeed,fsAngle};
        // std::vector<float> fspeed_temp = {0,0,0};
        fspeed_ref[i] = fspeed_temp;

               
         
        //usleep(100000);

    }

    if((t_current+dt)<=s/r/w){
            t_current=t_current+dt;

    }

    



}


void PLANNER::linear_traject_2(){
    
    std::cout << s <<std::endl;
    dt = sampleTime;

    t=t_current;
    

    for(int i =0;i<window;i++){

        if((t+dt)<=s/50/w){
            t=t+dt;

        }
        /////////////////////////////////////  pos  ////////////////////////////////////////
        float xt = 50*w*t;
        float yt = 0;

        std::vector<float> point = {xt,yt,0};
        pos_ref[i] = point;


        /////////////////////////////////////  vel  ////////////////////////////////////////
        float vt_x = 50;//5;
        float vt_y = 0;
        // float vt = sqrt(pow(vt_x,2)+pow(vt_y,2));

        // float vn_x = -r*w*sin(w*t);
        // float vn_y = r*w*cos(w*t);
        // float vn = sqrt(pow(vn_x,2)+pow(vn_y,2));

        float vx=vt_x;//vt_x;
        float vy=0;//vt_y;

        if((t+dt)>=s/50/w){
            vx=0;
            vy=0;
        //std::cout << "-----------------------------------------" <<std::endl;
        }

        //std::cout << vx    << " "<< vy <<std::endl;


        std::vector<float> vel = {vx,vy,0};
        // std::vector<float> vel = {0,0,0};
        vel_ref[i] = vel;



        //pos_ref.push_back(point);
        //std::cout << xt    << " "<< yt <<std::endl;
               
         
        //usleep(100000);

    }

    if((t_current+dt)<=s/50/w){
            t_current=t_current+dt;
            std::cout << "12345678998765432423546987456321123456789222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222" <<std::endl;

    }

    



}