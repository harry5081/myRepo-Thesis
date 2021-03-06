#include <fstream>
#include <vector>
#include <iostream>
#include <thread>
#include "../matplotlib-cpp-master/matplotlibcpp.h"

//g++ -o plot.out plotThreeDyn.cpp -I/usr/include/python2.7 /lib/x86_64-linux-gnu/libpthread.so.0 -lpython2.7


template<class T> void readFileToVector(std::string fileName, std::vector<T> &data);
void windowSizeControl();
void plotX();
void plot(std::string CAN_Write_Time, std::string CAN_Write_Data, std::string CAN_Read_Time, std::string CAN_Read_Data);
void intp();

int wSize = 20;

std::vector<float> posDemand_Time_temp;
std::vector<float> posDemand_X_temp;
std::vector<float> posDemand_Y_temp;
std::vector<float> posDemand_Z_temp;

std::vector<float> velDemand_Time_temp;
std::vector<float> velDemand_X_temp;
std::vector<float> velDemand_Y_temp;
std::vector<float> velDemand_Z_temp;



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
std::vector<float> canReadData_PosX_temp;
std::vector<float> canReadData_PosY_temp;
std::vector<float> canReadData_PosZ_temp;




bool intp_button = false;



//std::shared_ptr<std::thread> plotX_thread;
namespace plt = matplotlibcpp;

int main() {
    std::cout<<" Start Read File and Plot"<<std::endl;


    std::thread plot_thread(&plotX);
    std::thread intp_thread(&intp);
 
    
 
    plot_thread.join();
    intp_thread.join();
  
    return 0;
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
        }

        else if(temp == 'c'){
            intp_button = false;
        }

        sleep(1);
    }

}
    

void plotX(){
    
    
    plt::figure_size(1200, 780);
    
    plt::figure(1);
    
    plt::subplot(3,1,1);
    plt::title("Vel");
    plt::Plot plot0("VelDemandX",velDemand_Time_temp,velDemand_X_temp,"b--");   
    plt::Plot plot1("WritecanX",canWriteTime_temp,canWriteData_temp,"k--");
    plt::Plot plot2("WritereadX",canReadTime_temp,canReadData_temp,"r");
    
    plt::grid();


    plt::subplot(3,1,2);
    plt::Plot plot2_9("VelDemandY",velDemand_Time_temp,velDemand_Y_temp,"b--");
    plt::Plot plot3("WritecanY",canWriteTime_temp,canWriteData_Y_temp,"k--");
    plt::Plot plot4("WritereadY",canReadTime_temp,canReadData_Y_temp,"r");
   
    plt::grid();


    plt::subplot(3,1,3);
    plt::Plot plot4_5("VelDemandZ",velDemand_Time_temp,velDemand_Z_temp,"b--");
    plt::Plot plot4_6("WritecanZ",canWriteTime_temp,canWriteData_Z_temp,"k--");
    plt::Plot plot4_7("WritereadZ",canReadTime_temp,canReadData_Z_temp,"r");
   
    plt::grid();
    




    plt::figure(2);
    
    plt::subplot(3,1,1);
    plt::title("Pos");
       
    plt::Plot plot5("PosDemandX",posDemand_Time_temp,posDemand_X_temp,"b--");
    plt::Plot plot6("WritereadPosX",canReadTime_Pos_temp,canReadData_PosX_temp,"r");
    plt::grid(); 


    plt::subplot(3,1,2);
       
    plt::Plot plot7("PosDemandY",posDemand_Time_temp,posDemand_Y_temp,"b--");
    plt::Plot plot8("WritereadPosY",canReadTime_Pos_temp,canReadData_PosY_temp,"r");
    plt::grid(); 

    plt::subplot(3,1,3);
       
    plt::Plot plot9("PosDemandZ",posDemand_Time_temp,posDemand_Z_temp,"b--");
    plt::Plot plot10("WritereadPosZ",canReadTime_Pos_temp,canReadData_PosZ_temp,"r");
    plt::grid();

    


  

    while(true){
        if(intp_button==false){
        
        /////////////////////////////////////////////////////////////////////////////
        ///////////////////////////     velocity part     //////////////////////////
        ///////////////////////////////////////////////////////////////////////////// 
        readFileToVector("../build/plot/CAN_Write_Time", canWriteTime_temp);
        readFileToVector("../build/plot/CAN_Write_Data_X", canWriteData_temp);
        readFileToVector("../build/plot/CAN_Write_Data_Y", canWriteData_Y_temp);
        readFileToVector("../build/plot/CAN_Write_Data_Z", canWriteData_Z_temp);
        

        readFileToVector("../build/plot/CAN_Read_Time", canReadTime_temp);
        readFileToVector("../build/plot/CAN_Read_Data_X", canReadData_temp);
        readFileToVector("../build/plot/CAN_Read_Data_Y", canReadData_Y_temp);
        readFileToVector("../build/plot/CAN_Read_Data_Z", canReadData_Z_temp);
        


        readFileToVector("../build/plot/VelDemand_Time", velDemand_Time_temp);
        readFileToVector("../build/plot/VelDemand_Data_X", velDemand_X_temp);
        readFileToVector("../build/plot/VelDemand_Data_Y", velDemand_Y_temp);
        readFileToVector("../build/plot/VelDemand_Data_Z", velDemand_Z_temp);

        windowSizeControl();
           
        
        ///////////////////////////     velocity part  X   //////////////////////////
     
        if(canWriteTime_temp.size()==canWriteData_temp.size()){      
            
            plt::figure(1);
            plt::subplot(3,1,1);
            plt::xlim(0, wSize);
            plt::ylim(-150, 150);
            
            plot1.update(canWriteTime_temp,canWriteData_temp);

            if(canReadTime_temp.size()==canReadData_temp.size()){
               
                plot2.update(canReadTime_temp,canReadData_temp);
                
            }

            if(velDemand_Time_temp.size()==velDemand_X_temp.size()){
                
                plot0.update(velDemand_Time_temp,velDemand_X_temp);
                
            }
        }
         plt::pause(0.05);

        ///////////////////////////     velocity part  Y   //////////////////////////

        if(canWriteTime_temp.size()==canWriteData_Y_temp.size()){      
          
          plt::figure(1);
          plt::subplot(3,1,2);
          plt::xlim(0, wSize);
          plt::ylim(-150, 150);
         
          plot3.update(canWriteTime_temp,canWriteData_Y_temp);
          std::cout<<"OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"<<std::endl;
          if(canReadTime_temp.size()==canReadData_Y_temp.size()){
                plot4.update(canReadTime_temp,canReadData_Y_temp);
            }

          if(velDemand_Time_temp.size()==velDemand_Y_temp.size()){
                
                plot2_9.update(velDemand_Time_temp,velDemand_Y_temp);
          }      
           else{
                std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
           }     
          
        }
        plt::pause(0.05);


        ///////////////////////////     velocity part  Z   //////////////////////////

        if(canWriteTime_temp.size()==canWriteData_Z_temp.size()){      
            
            plt::figure(1);
            plt::subplot(3,1,3);
            plt::xlim(0, wSize);
            plt::ylim(-50, 50);
            
            plot4_6.update(canWriteTime_temp,canWriteData_Z_temp);

            if(canReadTime_temp.size()==canReadData_Z_temp.size()){
               
                plot4_7.update(canReadTime_temp,canReadData_Z_temp);
                
            }

            if(velDemand_Time_temp.size()==velDemand_Z_temp.size()){
                
                plot4_5.update(velDemand_Time_temp,velDemand_Z_temp);
                
            }
        }
         plt::pause(0.05); 


        

        /////////////////////////////////////////////////////////////////////////////
        ///////////////////////////     position part     ///////////////////////////
        /////////////////////////////////////////////////////////////////////////////

        readFileToVector("../build/plot/PosDemand_Time", posDemand_Time_temp);
        readFileToVector("../build/plot/PosDemand_Data_X", posDemand_X_temp);
        readFileToVector("../build/plot/PosDemand_Data_Y", posDemand_Y_temp);
        readFileToVector("../build/plot/PosDemand_Data_Z", posDemand_Z_temp);
        
        readFileToVector("../build/plot/CAN_Read_Time_Pos", canReadTime_Pos_temp);
        readFileToVector("../build/plot/CAN_Read_Data_PosX", canReadData_PosX_temp);
        readFileToVector("../build/plot/CAN_Read_Data_PosY", canReadData_PosY_temp);

        readFileToVector("../build/plot/CAN_Read_Time_PosZ", canReadTime_PosZ_temp);
        readFileToVector("../build/plot/CAN_Read_Data_PosZ", canReadData_PosZ_temp);


        ///////////////////////////     position part X    ///////////////////////////
       
        if(posDemand_Time_temp.size()==posDemand_X_temp.size()){      
            plt::figure(2);
            plt::subplot(3,1,1);
            plt::xlim(0, wSize);
            plt::ylim(-150, 150);
            //plt::ylim(-100, 200);

            plot5.update(posDemand_Time_temp,posDemand_X_temp);

            if(canReadTime_Pos_temp.size()==canReadData_PosX_temp.size()){
                plot6.update(canReadTime_Pos_temp,canReadData_PosX_temp);
            }
       
        }
        

        //plt::pause(0.05);


        ///////////////////////////     position part Y    ///////////////////////////

        if(posDemand_Time_temp.size()==posDemand_Y_temp.size()){      
            //std::cout<<"OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"<<std::endl;
            plt::figure(2);
            plt::subplot(3,1,2);
            plt::xlim(0, wSize);
            plt::ylim(-150, 150);
            //plt::ylim(-100, 200);

            plot7.update(posDemand_Time_temp,posDemand_Y_temp);

            if(canReadTime_Pos_temp.size()==canReadData_PosY_temp.size()){
                plot8.update(canReadTime_Pos_temp,canReadData_PosY_temp);
            }
       
        }
        else{
            //std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }

        //plt::pause(0.05);

        ///////////////////////////     position part Z    ///////////////////////////

        if(posDemand_Time_temp.size()==posDemand_Z_temp.size()){      
            //std::cout<<"OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"<<std::endl;
            plt::figure(2);
            plt::subplot(3,1,3);
            plt::xlim(0, wSize);
            //plt::ylim(-150, 150);
            plt::ylim(-60, 60);

            plot9.update(posDemand_Time_temp,posDemand_Z_temp);

            if(canReadTime_PosZ_temp.size()==canReadData_PosZ_temp.size()){
                plot10.update(canReadTime_PosZ_temp,canReadData_PosZ_temp);
            }

            else{
            std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            }
       
        }
        

        //plt::pause(0.05);



        //ref_vel
        velDemand_Time_temp.clear();
        velDemand_X_temp.clear();
        velDemand_Y_temp.clear();
        velDemand_Z_temp.clear();

        //ref_pos
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

        
        

        }//if(intp_button==false)
        
    }//while(true)

}


void plot(std::string CAN_Write_Time, std::string CAN_Write_Data, std::string CAN_Read_Time, std::string CAN_Read_Data){
    
   plt::Plot plot3("Writecan",canWriteTime_temp,canWriteData_temp,"k--");
    //plt::figure_size(1200, 780);
    //plt::grid();
    //plt::figure(2);
//plt::Plot plot;
    // plt::Plot plot1("Writecan",canWriteTime_Y_temp,canWriteData_temp,"k--");
    // plt::Plot plot2("Writeread",canReadTime_Y_temp,canReadData_temp,"r");

    // while(true){

    //     readFileToVector(CAN_Write_Time, canWriteTime_Y_temp);
    //     readFileToVector(CAN_Write_Data, canWriteData_Y_temp);

    //     readFileToVector(CAN_Read_Time, canReadTime_Y_temp);
    //     readFileToVector(CAN_Read_Data, canReadData_Y_temp);

    //     if(canWriteTime_Y_temp.size()==canWriteData_Y_temp.size()){
           
    //     //   plt::xlim(0, wSize);
    //     //   plt::ylim(-300, 300);

    //       plot.update(canWriteTime_Y_temp,canWriteData_Y_temp);

    //     //   if(canReadTime_Y_temp.size()==canReadData_Y_temp.size()){

    //     //         plot2.update(canReadTime_Y_temp,canReadData_Y_temp);
        
    //     //     }
    //     }
    
    //     plt::pause(0.05);
    //     canWriteTime_Y_temp.clear();
    //     canWriteData_Y_temp.clear();

    //     canReadData_Y_temp.clear();
    //     canReadTime_Y_temp.clear();
    //}

}

