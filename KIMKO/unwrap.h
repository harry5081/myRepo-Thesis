#ifndef UNWRAP_H
#define UNWRAP_H

#include <cmath>
#define M_2PI M_PI*2

#define ANGLE_360 360
#define ANGLE_180 180

// https://stackoverflow.com/questions/15634400/continous-angles-in-c-eq-unwrap-function-in-matlab?fbclid=IwAR3D_bVMl22gOn-KFuW5aKUhKWXniYnTY5tOUFQ9Gcu3X1np3c954ICuTK4

//Normalize to [-180,180):
inline double constrainAngle(double x){
    x = fmod(x + ANGLE_180,ANGLE_360);
    if (x < 0)
        x += ANGLE_360;
    return x - ANGLE_180;
}


// convert to [-360,360]
inline double angleConv(double angle){
    return fmod(constrainAngle(angle),ANGLE_360);
}


inline double angleDiff(double a,double b){
    double dif = fmod(b - a + ANGLE_180,ANGLE_360);
    if (dif < 0)
        dif += ANGLE_360;
    return dif - ANGLE_180;
}


inline double unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,angleConv(previousAngle));
}



//Normalize to [-180,180):
inline double constrainAngleRad(double x){
    x = fmod(x + M_PI,M_2PI);
    if (x < 0)
        x += M_2PI;
    return x - M_PI;
}


// convert to [-360,360]
inline double angleConvRad(double angle){
    return fmod(constrainAngleRad(angle),M_2PI);
}


inline double angleDiffRad(double a,double b){
    double dif = fmod(b - a + M_PI,M_2PI);
    if (dif < 0)
        dif += M_2PI;
    return dif - M_PI;
}


inline double unwrapRad(double previousAngle,double newAngle){
    return previousAngle - angleDiffRad(newAngle,angleConvRad(previousAngle));
}


#endif
