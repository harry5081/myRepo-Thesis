#include "traj_plan.h"
#include <vector>

PLANNER::PLANNER(){
    
    r = leaderData.r;
    offset_x=leaderData.offset_xs;
    offset_y=leaderData.offset_ys;

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
	std::string file = "traj.txt";
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
			
            // getline(traj, t, '\t');
			// index_f.push_back(std::stoi(t));

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
		std::cout  << x_Pos_f[j] << "\t" << y_Pos_f[j] << "\t" << phi_ref_f[j] << "\t" << s_Dot_f[j] << "\t" << blank_ref_f[j]<< "\t" << w_ref_f[j]<< "\t"<< ori_ref_f[j] << std::endl;
		
	}
	std::cout << std::endl;

}

void PLANNER::traject_from_file(){
   
    dt = 1;
    t=t_current;
    
    //fsAngle_pre_window = fsAngle_pre; /********* angle unwrap *********/
    
    //within prediction horizon
    for(int i =0;i<window;i++){
            
        /////////////////////////////////////  pos  ////////////////////////////////////////
        float xt = x_Pos_f[t];
        float yt = y_Pos_f[t];
        float zt = ori_ref_f[t];
               
        /////////////////////////////////////  forward speed  ////////////////////////////////////////
        float fspeed = s_Dot_f[t]/sampleTime;
        float fsAngle = phi_ref_f[t]; //rads
        float ws = w_ref_f[t]/sampleTime;
        

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
 
       
        std::vector<float> pos = {xt,yt,fsAngle};
        pos_ref[i] = pos;

        std::vector<float> vel = {fspeed,0,ws};
        vel_ref[i] = vel;

        std::vector<float> ori = {zt,0};
        ori_ref[i] = ori;

        std::vector<float> guess_temp = {xt,yt,fsAngle,fspeed,0,0};
        guess[i] = guess_temp;
               
    }


    //MPC iteration
    if((t_current+dt)<(lineCount-1)){
            t_current=t_current+dt;

    }

    

}


void PLANNER::cir_traject_TNB(){
    
    dt = ds;
    float s_dot = dt/sampleTime;

    float k = 1/r;
    float ws =0;

    float ori_temp=0;

    t=t_current;
    fsAngle_pre_window = fsAngle_pre; /********* angle unwrap *********/
    
    // within prediction horizon
    for(int i =0;i<window;i++){

        if((t+dt)<=s){
            t=t+dt;
        }

        ws =k*s_dot;
        /////////////////////////////////////  pos  ////////////////////////////////////////
        float xt = r*sin(t/r)+offset_x;     //clockwise
        float yt = r*-cos(t/r)+r+offset_y;

        //float xt = r*sin(t/r)+offset_x;       //counter clockwise
        //float yt = r*cos(t/r)-r+offset_y;
                
        float zt =0;


        /////////////////////////////////////  vel  ////////////////////////////////////////
        float vt_x = cos(t/r);          //clockwise
        float vt_y = sin(t/r);

        //float vt_x = cos(t/r);        //counter clockwise
        //float vt_y = -sin(t/r);
        //float vt = sqrt(pow(vt_x,2)+pow(vt_y,2));

        /////////////////////////////////////  forward speed  ////////////////////////////////////////
        float fspeed = s_dot;
        float fsAngle = atan2(vt_y,vt_x);

        /********* angle unwrap *********/
        float fsAngle_unwrap = unwrapRad(fsAngle_pre_window,fsAngle);
        fsAngle_pre_window=fsAngle_unwrap;

        if(i==0){

            fsAngle_pre = fsAngle_pre_window;
        }
        /********* angle unwrap *********/

        /////////////////////////////////////  ori  ////////////////////////////////////////
        ori_temp=t/dt*3;
        if(ori_temp>desireOri){
            ori_temp=desireOri;
        }

        //ori_temp=fsAngle_2PI*180.0/PI; heading of robot = reference_psi

        
        float vx=vt_x;//vt_x;//vt_x;
        float vy=vt_y;//vt_y;

        if((t+dt)>s){
            vx=0;
            vy=0;

            fspeed=0;
           
            zt =0;
            ws =0;
            ori_temp = desireOri;

        }

        if(t==0){
            xt=0;
            yt=0;
            ori_temp =0;
            fspeed=0;
            ws =0;
  
        }

        fsAngle_2PI = fmod(fsAngle_unwrap,M_2PI);
        
        std::vector<float> pos = {xt,yt,fsAngle_2PI};
        pos_ref[i] = pos;

        std::vector<float> vel = {fspeed,0,ws};
        vel_ref[i] = vel;

        std::vector<float> ori = {ori_temp,0};
        ori_ref[i] = ori;

        std::vector<float> guess_temp = {xt,yt,fsAngle_2PI,fspeed,0,0};
        guess[i] = guess_temp;
         
    }

    // MPC iteration
    if((t_current+dt)<=s){
            t_current=t_current+dt;
           
    }

    

}
