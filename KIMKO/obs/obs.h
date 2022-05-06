#ifndef OBS_H
#define OBS_H

#include <vector>

class OBS
{
private:
    /* data */
public:
    OBS(int x, int y, int r);
    //~OBS();

    int obs_x;
    int obs_y;

    int obs_r;

    std::vector<float> obs_bound_x;
    std::vector<float> obs_bound_y;

    std::vector<int> data;

};








#endif