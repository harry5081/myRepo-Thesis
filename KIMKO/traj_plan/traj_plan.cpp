#include "traj_plan.h"
#include <vector>

PLANNER::PLANNER(){
    
    r = leaderData.r;

    std::vector<float> temp={0,0,0};

    std::vector<float> temp_ori={0,0};
    std::vector<float> temp_guess={0,0,0,0,0,0};
    
     for(int w =0;w<window;w++)
     {

        pos_ref.push_back(temp);
        vel_ref.push_back(temp);
        fspeed_ref.push_back(temp);

        ori_ref.push_back(temp_ori);
        guess.push_back(temp_guess);
     }

    readTrajFile();

}

void PLANNER::readTrajFile(){
    
    std::cout << "Planner read traj file " << std::endl;
    // define variables
	std::string t, xPos, yPos, phi, sDot, blank, w, ori; //variables from file are here
	
	//input filename
	std::string file = "traj2.txt";
	std::string str;
	//number of lines
	int i = 0;

    // open file and read to know how many lines are there in advance

    std::ifstream temp(file); //opening the file.
    if (temp.is_open()){
        std::string tp;

        while(getline(temp, tp)){ //read data from file object and put it into string.
            
            lineCount++;
            
        }

        //std::cout << lineCount << "\n"; //print the data of the string
        temp.close(); //close the file object.
    }

    



	std::ifstream traj(file); //opening the file.
	
    if (traj.is_open()) //if the file is open
	{
		//ignore first line
		std::string line;
		std::getline(traj, line);

		while(i<lineCount-1) //while the end of file is NOT reached
		{
            i++;
			
            getline(traj, t, '\t');
			index_f.push_back(std::stoi(t));

			getline(traj, xPos, '\t');
			x_Pos_f.push_back(stof(xPos));

			getline(traj, yPos, '\t');
			y_Pos_f.push_back(stof(yPos));

			getline(traj, phi, '\t');
            phi_ref_f.push_back(stof(phi));

			getline(traj, sDot, '\t');
			s_Dot_f.push_back(stof(sDot));

            getline(traj, blank, '\t');
			blank_ref_f.push_back(stof(blank));

            getline(traj, w, '\t'); //new line 
			w_ref_f.push_back(stof(w));

            getline(traj, ori, '\n'); //new line 
			ori_ref_f.push_back(stof(ori));
          	
		}
		traj.close(); //closing the file
		std::cout << "Number of entries: " << lineCount-1 << std::endl;
	}
	else {
        std::cout << "Unable to open file in PLANNER"; //if the file is not open output
    }
	
    
	for (int j = 0; j < i; j++) {
		std::cout << index_f[j] << "\t" << x_Pos_f[j] << "\t" << y_Pos_f[j] << "\t" << phi_ref_f[j] << "\t" << s_Dot_f[j] << "\t" << blank_ref_f[j]<< "\t" << w_ref_f[j]<< "\t"<< ori_ref_f[j] << std::endl;
		
	}
	std::cout << std::endl;

}
void PLANNER::traject_from_file(){
   
    dt = 1;
    t=t_current;
    
    //fsAngle_pre_window = fsAngle_pre; /********* angle unwrap *********/
    
    for(int i =0;i<window;i++){

        
        /////////////////////////////////////  pos  ////////////////////////////////////////
        float xt = x_Pos_f[t];
        float yt = y_Pos_f[t];
        float zt = ori_ref_f[t];
        //std::cout << t <<std::endl;

        /////////////////////////////////////  vel  ////////////////////////////////////////
        // float vt_x = cos(t/r);
        // float vt_y = sin(t/r);
        // //float vt = sqrt(pow(vt_x,2)+pow(vt_y,2));

        // float vn_x = -(1/r)*sin(t/r);
        // float vn_y = (1/r)*cos(t/r);
        // float vn = sqrt(pow(vn_x,2)+pow(vn_y,2));

        /////////////////////////////////////  forward speed  ////////////////////////////////////////
        // float fspeed = sqrt(pow(vt_x,2)+pow(vt_y,2));
        float fspeed = s_Dot_f[t]/sampleTime;
        float fsAngle = phi_ref_f[t];
        float ws = w_ref_f[t]/sampleTime;
        //std::cout << fsAngle <<std::endl;
        //float fspeed = 40;
        //float fsAngle = atan(vt_y/vt_x);
        //float fsAngle = atan2(vt_y,vt_x);

        
        
        
        /********* angle unwrap *********/
        // float fsAngle_unwrap = unwrapRad(fsAngle_pre_window,fsAngle);
        // fsAngle_pre_window=fsAngle_unwrap;

        // if(i==0){

        //     fsAngle_pre = fsAngle_pre_window;
        // }
        /********* angle unwrap *********/
     
        // float vx=vt_x;//vt_x;//vt_x;
        // float vy=vt_y;//vt_y;

        if((t+dt)<(lineCount-1)){
            t=t+dt;
  
        }

        if((t+dt)>=(lineCount-1)){  // reach the end of trajectory
            // vx=0;
            // vy=0;

            fspeed=0;
            // fsAngle=0;
            // fsAngle_unwrap=0;
            //zt =0;
            ws =0;
        
        }

        // if(t==0){
        //     xt=0;
        //     yt=0;

            
        // }


        //fsAngle_2PI = fmod(fsAngle,M_2PI);
        //std::vector<float> pos = {0,0,0};
        //std::vector<float> pos = {200,0,0};
        //std::vector<float> pos = {-300,300,3*PI/4};
        std::vector<float> pos = {xt,yt,fsAngle};
        //std::vector<float> pos = {xt,yt,fsAngle_2PI};
        pos_ref[i] = pos;

        std::vector<float> vel = {fspeed,0,ws};
        //std::vector<float> vel = {0,0,0};
        //std::vector<float> vel = {25,0,ws};
        vel_ref[i] = vel;

        std::vector<float> ori = {zt,0};
        ori_ref[i] = ori;

        std::vector<float> guess_temp = {xt,yt,fsAngle,fspeed,0,0};
        //std::vector<float> guess_temp = {200,0,0,0,0,0};
        guess[i] = guess_temp;
               
         
        //usleep(100000);

    }

    if((t_current+dt)<(lineCount-1)){
            t_current=t_current+dt;

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



        //std::vector<float> point = {xt,0,0};
        std::vector<float> point = {xt,yt,0};
        pos_ref[i] = point;


        //std::vector<float> vel = {vx,0,0};
         std::vector<float> vel = {0,0,0};
        vel_ref[i] = vel;

        //std::vector<float> fspeed_temp = {fspeed,fsAngle};
        //std::vector<float> fspeed_temp = {fspeed,fsAngle};
         std::vector<float> fspeed_temp = {0,0,0};
        fspeed_ref[i] = fspeed_temp;

               
         
        //usleep(100000);

    }

    if((t_current+dt)<=s/r/w){
            t_current=t_current+dt;

    }

    



}

void PLANNER::cir_traject_TNB(){
    // std::cout << "TNB" <<std::endl;
    // std::cout << s <<std::endl;
    dt = 15;
    float s_dot = dt/sampleTime;
    float k = 1/r;
    float ws =0;

    float ori_temp=0;

    t=t_current;
    fsAngle_pre_window = fsAngle_pre; /********* angle unwrap *********/
    //std::cout << t <<std::endl;
    for(int i =0;i<window;i++){

        if((t+dt)<=s){
            t=t+dt;
  
        }

        ws =k*s_dot;
        /////////////////////////////////////  pos  ////////////////////////////////////////
        float xt = r*sin(t/r);
        float yt = r*-cos(t/r)+r;
        float zt =0;


        // if((t+dt)<=s){
        // xt = 200;
        // }

        /////////////////////////////////////  vel  ////////////////////////////////////////
        float vt_x = cos(t/r);
        float vt_y = sin(t/r);
        //float vt = sqrt(pow(vt_x,2)+pow(vt_y,2));

        float vn_x = -(1/r)*sin(t/r);
        float vn_y = (1/r)*cos(t/r);
        float vn = sqrt(pow(vn_x,2)+pow(vn_y,2));


        /////////////////////////////////////  forward speed  ////////////////////////////////////////
        // float fspeed = sqrt(pow(vt_x,2)+pow(vt_y,2));
        float fspeed = s_dot;
        //float fspeed = 40;
        //float fsAngle = atan(vt_y/vt_x);
        float fsAngle = atan2(vt_y,vt_x);

        /////////////////////////////////////  ori  ////////////////////////////////////////
        ori_temp=t/dt*2;
        if(ori_temp>desireOri){
            ori_temp=desireOri;
        }
        
        
        /********* angle unwrap *********/
        float fsAngle_unwrap = unwrapRad(fsAngle_pre_window,fsAngle);
        fsAngle_pre_window=fsAngle_unwrap;

        if(i==0){

            fsAngle_pre = fsAngle_pre_window;
        }
        /********* angle unwrap *********/

        
        float vx=vt_x;//vt_x;//vt_x;
        float vy=vt_y;//vt_y;

        if((t+dt)>s){
            vx=0;
            vy=0;

            fspeed=0;
            fsAngle=0;
            fsAngle_unwrap=0;

            zt =0;
            ws =0;
            ori_temp=desireOri;

        }

        if(t==0){
            xt=0;
            yt=0;
            ori_temp =0;
            fspeed=0;
            ws =0;
  
        }
        fsAngle_2PI = fmod(fsAngle_unwrap,M_2PI);
        //std::vector<float> pos = {0,0,0};
        //std::vector<float> pos = {0,300,0};
        //std::vector<float> pos = {-300,300,3*PI/4};
        std::vector<float> pos = {xt,yt,fsAngle_2PI};
        //std::vector<float> pos = {xt,yt,fsAngle_2PI};
        pos_ref[i] = pos;

        std::vector<float> vel = {fspeed,0,ws};
        //std::vector<float> vel = {0,0,0};
        //std::vector<float> vel = {25,0,ws};
        vel_ref[i] = vel;

        std::vector<float> ori = {ori_temp,0};
        ori_ref[i] = ori;

        std::vector<float> guess_temp = {xt,yt,fsAngle_2PI,fspeed,0,0};
        //std::vector<float> guess_temp = {0,300,0,0,0,0};
        guess[i] = guess_temp;
         

    }

    if((t_current+dt)<=s){
            t_current=t_current+dt;

    }

    



}

void PLANNER::cir_traject_TNB_preAngle(){
    // std::cout << "TNB" <<std::endl;
    // std::cout << s <<std::endl;
    dt = 10;
    float s_dot = dt/sampleTime;
    float k = 1/r;
    float ws =0;

    //float desireOri = 0;

    if(pre_s<s_pre_route){ // turn the ori before running the route
        pre_s=pre_s+dt;
        for(int i =0;i<window;i++){
            //std::vector<float> pos = {xt,yt,fsAngle};
            std::vector<float> pos = {0,0,0};
            //std::vector<float> pos = {xt,yt,fsAngle};
            // std::vector<float> pos = {xt,yt,0};
            pos_ref[i] = pos;

            std::vector<float> vel = {0,0,0};
            //std::vector<float> vel = {0,0,0};
            //std::vector<float> vel = {25,0,ws};
            vel_ref[i] = vel;

            std::vector<float> ori = {desireOri,0};
            ori_ref[i] = ori;
        }

    }

    else{
  
        t=t_current;
        fsAngle_pre_window = fsAngle_pre; /********* angle unwrap *********/
        //std::cout << t <<std::endl;
        for(int i =0;i<window;i++){

            if((t+dt)<=s){
                t=t+dt;
    
            }

            ws =k*s_dot;
            /////////////////////////////////////  pos  ////////////////////////////////////////
            float xt = r*sin(t/r);
            float yt = r*-cos(t/r)+r;
            float zt =0;


            // if((t+dt)<=s){
            // xt = 200;
            // }

            /////////////////////////////////////  vel  ////////////////////////////////////////
            float vt_x = cos(t/r);
            float vt_y = sin(t/r);
            //float vt = sqrt(pow(vt_x,2)+pow(vt_y,2));

            float vn_x = -(1/r)*sin(t/r);
            float vn_y = (1/r)*cos(t/r);
            float vn = sqrt(pow(vn_x,2)+pow(vn_y,2));


            /////////////////////////////////////  forward speed  ////////////////////////////////////////
            // float fspeed = sqrt(pow(vt_x,2)+pow(vt_y,2));
            float fspeed = s_dot;
            //float fspeed = 40;
            //float fsAngle = atan(vt_y/vt_x);
            float fsAngle = atan2(vt_y,vt_x);
            
            /********* angle unwrap *********/
            float fsAngle_unwrap = unwrapRad(fsAngle_pre_window,fsAngle);
            fsAngle_pre_window=fsAngle_unwrap;

            if(i==0){

                fsAngle_pre = fsAngle_pre_window;
            }
            /********* angle unwrap *********/

            
            float vx=vt_x;//vt_x;//vt_x;
            float vy=vt_y;//vt_y;

            if((t+dt)>s){
                vx=0;
                vy=0;

                fspeed=0;
                fsAngle=0;
                fsAngle_unwrap=0;

                zt =0;
                ws =0;

            }

            if(t==0){
                xt=0;
                yt=0;
    
            }
            fsAngle_2PI = fmod(fsAngle_unwrap,M_2PI);
            std::vector<float> pos = {200,0,0};
            //std::vector<float> pos = {xt,yt,fsAngle_2PI};
            //std::vector<float> pos = {xt,yt,fsAngle};
            // std::vector<float> pos = {xt,yt,0};
            pos_ref[i] = pos;

            //std::vector<float> vel = {fspeed,0,ws};
            std::vector<float> vel = {0,0,0};
            //std::vector<float> vel = {25,0,ws};
            vel_ref[i] = vel;

            std::vector<float> ori = {desireOri,0};
            ori_ref[i] = ori;

            //std::vector<float> guess_temp = {xt,yt,fsAngle_2PI,fspeed,0,0};
            std::vector<float> guess_temp = {200,0,0,0,0,0};
            guess[i] = guess_temp;
            
        }

        if((t_current+dt)<=s){
                t_current=t_current+dt;

        }
    
    }// if(pre_s<s_pre_route) else
    



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