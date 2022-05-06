#include "obs.h"
#include <math.h>

OBS::OBS(int x, int y, int r)
{
    obs_x = x;
    obs_y = y;

    obs_r = r;

    float dt= 0.1;

    while(dt<=2*3.14){
        obs_bound_x.push_back(obs_r*cos(dt)+obs_x);
        obs_bound_y.push_back(obs_r*sin(dt)+obs_y);

        dt=dt+0.1;

    }

    data.push_back(obs_x);
    data.push_back(obs_y);
    data.push_back(obs_r);
}

// OBS::~obs()
// {
// }