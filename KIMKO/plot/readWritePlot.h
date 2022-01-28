#ifndef READ_WRITE_PLOT_H
#define READ_WRITE_PLOT_H

#include <stdio.h>
#include <iostream>
#include <fstream>


class ReadWritePlot{

private:
    //static bool writeFromEnd = false;
    

public:
    bool writeFromEnd;

    
    template <class T>
    void writeDatatoFile(T data, std::string fileName){

        std::ofstream file;
        
        if(writeFromEnd==false){
            file.open(fileName);
            writeFromEnd=true;
        }
        else{
           file.open(fileName,std::ofstream::app);
        }
        
        if(!file.is_open())
        {
            std::cout << "Failed to open " << fileName << "!\n";
            return;
        }
        else{
        
            file<<data<<std::endl;
        }
    
	file.close();
    }

    


};

#endif