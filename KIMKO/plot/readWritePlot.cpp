#include <fstream>
#include <vector>
#include <iostream>
#include "../matplotlib-cpp-master/matplotlibcpp.h"

//void DataPlot();

template<class T> void readFileToVector(std::string fileName, std::vector<T> &data);
void dataPlot(std::vector<int> &time, std::vector<int> &value);

int main(){
    std::cout<<" Start Read File and Plot"<<std::endl;
    namespace plt = matplotlibcpp;
    
    int wSize = 20;
    
    //(plt::backend_bases.event.key){
    std::vector<float> canWriteTime_X;
    std::vector<float> canWriteData_X;

    std::vector<float> canReadTime_X;
    std::vector<int> canReadData_X;

    //std::vector<float> canWriteTime_temp;
    //std::vector<float> canWriteData_temp;


    readFileToVector("../build/plot/CAN_Write_Time", canWriteTime_X);
    readFileToVector("../build/plot/CAN_Write_Data_X", canWriteData_X);
   

    readFileToVector("../build/plot/CAN_Read_Time", canReadTime_X);
    readFileToVector("../build/plot/CAN_Read_Data_X", canReadData_X);


    namespace plt = matplotlibcpp;
    
    //plt::ion();
    
    //plt::subplot(1,3,1);    
    //plt::plot(canReadTime_X,canReadData_X,"r-",{{"label","Output"}});
    // plt::xlim(0, 300);
    // plt::ylim(0, 300);
    // plt::plot(canWriteTime_X,canWriteData_X,"k--",{{"label","Input"}});
    // plt::legend();
    // plt::title("X_Velocity");
    // plt::show();
    // sleep(1);
    
    
    //    readFileToVector("CAN_Write_Time_X", canWriteTime_X);
    //readFileToVector("CAN_Write_Data_X", canWriteData_X);

    //  std::vector<float> canWriteTime_X1;
    // std::vector<int> canWriteData_X1;
    

    // readFileToVector("CAN_Write_Time_X", canWriteTime_temp);
    // readFileToVector("CAN_Write_Data_X", canWriteData_temp);

    //plot1.update(canWriteTime_temp,canWriteData_temp);
   //plt::pause(1);
    //plot1.clear();
    //plt::pause(1);

    std::vector<float> canWriteTime_temp;
    std::vector<float> canWriteData_temp;

    std::vector<float> canReadTime_temp;
    std::vector<float> canReadData_temp;

    plt::figure_size(1200, 780);
    plt::grid();

    plt::Plot plot1("Writecan",canWriteTime_temp,canWriteData_temp,"k--");
    plt::Plot plot2("Writeread",canReadTime_temp,canReadData_temp,"r");

    

while(true){

    
    //plt::plot(canWriteTime_X,canWriteData_X,"k--",{{"label","Input"}});

    

    readFileToVector("../build/plot/CAN_Write_Time", canWriteTime_temp);
    readFileToVector("../build/plot/CAN_Write_Data_X", canWriteData_temp);

    readFileToVector("../build/plot/CAN_Read_Time", canReadTime_temp);
    readFileToVector("../build/plot/CAN_Read_Data_X", canReadData_temp);

    

    if(canWriteTime_temp.size()==canWriteData_temp.size()){
       
        if(*(canWriteTime_temp.end()-1)>=10 && *(canWriteTime_temp.end()-1) >= wSize -10 ){
            wSize = int(*(canWriteTime_temp.end()-1)*2);
        }

        else if( *(canWriteTime_temp.end()-1)<10){

            wSize=20;
        }

        plt::xlim(0, wSize);
        plt::ylim(-300, 300);
    

        plot1.update(canWriteTime_temp,canWriteData_temp);
        

        if(canReadTime_temp.size()==canReadData_temp.size()){

        plot2.update(canReadTime_temp,canReadData_temp);
        
        }

        

        


    }

    plt::pause(0.05);
    canWriteTime_temp.clear();
    canWriteData_temp.clear();

    canReadData_temp.clear();
    canReadTime_temp.clear();

//wSize = wSize+5;
    

    //plot1.clear();
     //plt::pause(0.5);
//readFileToVector("CAN_Write_Time_X", canWriteTime_temp);
    //readFileToVector("CAN_Write_Data_X", canWriteData_temp);
    

    //readFileToVector("CAN_Write_Time_X", canWriteTime_X1);
    //readFileToVector("CAN_Write_Data_X", canWriteData_X1);

    //std::vector<int> intVec(canWriteTime_X1.begin(), canWriteTime_X1.end());
    
    //plot1.plot(intVec,canWriteData_X);
    //auto fig = plt::figure();
    
    

   //plot1.update(canWriteTime_temp,canWriteData_temp);
    //plt::pause(1);
    //plt::show();
    //sleep(1);
    //plot1.clear();

    
}
/*
    //std::vector<float> canWriteTime_Y;
    std::vector<int> canWriteData_Y;

    //std::vector<float> canReadTime_Y;
    std::vector<int> canReadData_Y;


    //readFileToVector("CAN_Write_Time_Y", canWriteTime_Y);
    readFileToVector("CAN_Write_Data_Y", canWriteData_Y);
   

    //readFileToVector("CAN_Read_Time_Y", canReadTime_Y);
    readFileToVector("CAN_Read_Data_Y", canReadData_Y);


    namespace plt = matplotlibcpp;

    plt::subplot(1,3,2);      
    plt::plot(canReadTime_X,canReadData_Y,"r-",{{"label","Output"}});
    plt::plot(canWriteTime_X,canWriteData_Y,"k--",{{"label","Input"}});
    plt::legend();
    plt::title("Y_Velocity");
    //plt::show();

    //std::vector<float> canWriteTime_Z;
    std::vector<float> canWriteData_Z;

    //std::vector<float> canReadTime_Z;
    std::vector<float> canReadData_Z;


    //readFileToVector("CAN_Write_Time_Z", canWriteTime_Z);
    readFileToVector("CAN_Write_Data_Z", canWriteData_Z);
   

    //readFileToVector("CAN_Read_Time_Z", canReadTime_Z);
    readFileToVector("CAN_Read_Data_Z", canReadData_Z);


    namespace plt = matplotlibcpp;

    plt::subplot(1,3,3);    
    plt::plot(canReadTime_X,canReadData_Z,"r-",{{"label","Output"}});
    plt::plot(canWriteTime_X,canWriteData_Z,"k--",{{"label","Input"}});
    plt::legend();
    plt::title("Z_Velocity");
    
    plt::show();
 */   
    



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

void dataPlot(std::vector<int> &time, std::vector<int> &value){

    namespace plt = matplotlibcpp;

    plt::plot(time,value);
    plt::show();


}


