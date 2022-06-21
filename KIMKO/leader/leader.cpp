#include "leader.h"
#include <math.h>

LEADER::LEADER(){

    std::cout <<  "LEADER init"<<std::endl;

    std::vector<float> temp={0,0};
    
    //  for(int i=0 ;i<= point_cnt; i++)
    //  {

        curve.push_back(temp);
        

     //}

    
    
    
    
}

void LEADER::file_traject_init(){
    readTrajFile();

}

void LEADER::cir_traject_init(){
    for(int i=0;i<point_cnt;i++){

        float xs_temp = r*sin(s/r)+offset_xs;
        float ys_temp = r*-cos(s/r)+r+offset_ys;

        std::vector<float> point = {xs_temp,ys_temp};
        s=s+ds;

        curve.push_back(point);
        //curve[i]=point;
    }

    float pre_time=0;

    while(1){

        //usleep(100000);  
        float time = (float)clock()/CLOCKS_PER_SEC;
        
        if(time>=startTime){

            if(time-pre_time>=period){
            
                cir_traject();
                pre_time = time;

                if(index>=point_cnt){
                    return;
                }
                

            }

        }//if(time>=startTime)

    }//while(1){
    
}

void LEADER::cir_traject(){

    //index = int((time-startTime)/period);
        
    if(index<point_cnt){
        s_current = index*ds;
        xs =curve[index][0];
        ys =curve[index][1];
        index++;
        // std::cout << "Leader: "<< s_current <<std::endl;
        // std::cout << "Leader: "<< xs<<  " "<<  ys<<std::endl;
        // std::cout << time <<std::endl;
        
    }


}


void LEADER::readTrajFile(){
    
    // define variables
	std::string t, xPos, yPos, phi, sDot, blank, w; //variables from file are here
	
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

        std::string garbage;

        float xs_temp;
        float ys_temp;

        s=s+ds;

        

		while(i<lineCount-1) //while the end of file is NOT reached
		{
            i++;
			
            // getline(traj, garbage, '\t');
		
			getline(traj, xPos, '\t');
            xs_temp = stof(xPos);

            getline(traj, yPos, '\t');
            ys_temp = stof(yPos);

            std::vector<float> point = {xs_temp,ys_temp};
            
            //curve[i]=point;
            curve.push_back(point);
			
			getline(traj, garbage, '\n');
            
          	
		}

		traj.close(); //closing the file
		std::cout << "Number of entries: " << lineCount-1 << std::endl;
        point_cnt = lineCount-1;
	}
	else {
        std::cout << "Unable to open file in LEADER"; //if the file is not open output
    }
	
    
	// for (int j = 0; j < i; j++) {
	// 	std::cout << index_f[j] << "\t" << x_Pos_f[j] << "\t" << y_Pos_f[j] << "\t" << phi_ref_f[j] << "\t" << s_Dot_f[j] << "\t" << blank_ref_f[j]<< "\t" << w_ref_f[j] << std::endl;
		
	// }
	// std::cout << std::endl;

}

