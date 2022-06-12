#include "readWritePlot.h"

///////////////////////////////////////////////////////////////////////////////////////////////
// plotting
// obs
ReadWritePlot *obs_Plot_PosX = new ReadWritePlot;
ReadWritePlot *obs_Plot_PosY = new ReadWritePlot;

// leader
//ReadWritePlot *leader_Plot_time = new ReadWritePlot;
ReadWritePlot *leader_Plot_PosX = new ReadWritePlot;
ReadWritePlot *leader_Plot_PosY = new ReadWritePlot;

// planner
//ReadWritePlot *leader_Plot_time = new ReadWritePlot;
ReadWritePlot *planner_Plot_PosX = new ReadWritePlot;
ReadWritePlot *planner_Plot_PosY = new ReadWritePlot;

// predic horz
ReadWritePlot *predicHorz_PosX = new ReadWritePlot;
ReadWritePlot *predicHorz_PosY = new ReadWritePlot;

// ref
ReadWritePlot *ref_Plot_Vel_time = new ReadWritePlot;
ReadWritePlot *ref_Plot_Pos_time = new ReadWritePlot;

// ref forward speed
ReadWritePlot *ref_Plot_fspeed = new ReadWritePlot;
ReadWritePlot *ref_Plot_fsAngle = new ReadWritePlot;

// robot forward speed
ReadWritePlot *robot_Plot_fspeed = new ReadWritePlot;
ReadWritePlot *robot_Plot_fsAngle = new ReadWritePlot;

// plot ref_X_Vel
ReadWritePlot *ref_Plot_VelX= new ReadWritePlot;
// plot ref_Y_Vel
ReadWritePlot *ref_Plot_VelY= new ReadWritePlot;
// plot ref_Z_Vel
ReadWritePlot *ref_Plot_VelZ= new ReadWritePlot;

// plot ref_X_Pos
ReadWritePlot *ref_Plot_PosX= new ReadWritePlot;
// plot ref_Y_Pos
ReadWritePlot *ref_Plot_PosY= new ReadWritePlot;
// plot ref_Z_Pos
ReadWritePlot *ref_Plot_PosZ= new ReadWritePlot;


// plot demand_TIME
ReadWritePlot *vd_Plot_Vel_time = new ReadWritePlot;
ReadWritePlot *pd_Plot_Pos_time = new ReadWritePlot;

// plot demand_X_Vel
ReadWritePlot *vd_Plot_VelX= new ReadWritePlot;
// plot demand_Y_Vel
ReadWritePlot *vd_Plot_VelY= new ReadWritePlot;
// plot demand_Z_Vel
ReadWritePlot *vd_Plot_VelZ= new ReadWritePlot;

ReadWritePlot *fsd_Vel_Plot= new ReadWritePlot;
ReadWritePlot *fsd_Angle_Plot= new ReadWritePlot;

// plot demand_X_Pos
ReadWritePlot *pd_Plot_PosX= new ReadWritePlot;
// plot demand_Y_Pos
ReadWritePlot *pd_Plot_PosY= new ReadWritePlot;
// plot demand_Z_Pos
ReadWritePlot *pd_Plot_PosZ= new ReadWritePlot;

//CAN READ, CAN WRITE TIME
ReadWritePlot *canWriteTimePlot_X = new ReadWritePlot;
ReadWritePlot *canReadTimePlot_X = new ReadWritePlot;

// plot Vel_X
ReadWritePlot *canWriteDataPlot_X = new ReadWritePlot;
ReadWritePlot *canReadDataPlot_X = new ReadWritePlot;

// plot Vel_Y
ReadWritePlot *canWriteDataPlot_Y = new ReadWritePlot;
ReadWritePlot *canReadDataPlot_Y = new ReadWritePlot;


// plot Vel_Z
ReadWritePlot *canWriteDataPlot_Z = new ReadWritePlot;
ReadWritePlot *canReadDataPlot_Z = new ReadWritePlot;


// plot Pos_TIME
ReadWritePlot *canReadTimePlot_PosX = new ReadWritePlot;
ReadWritePlot *canReadTimePlot_PosZ = new ReadWritePlot;

// plot Pos_X
ReadWritePlot *canReadDataPlot_PosX= new ReadWritePlot;
// plot Pos_Y
ReadWritePlot *canReadDataPlot_PosY= new ReadWritePlot;
// plot Pos_Z
ReadWritePlot *canReadDataPlot_PosZ= new ReadWritePlot;

// plot Pos_X_correct
ReadWritePlot *canReadDataPlot_PosX_cor = new ReadWritePlot;
// plot Pos_Y_correct
ReadWritePlot *canReadDataPlot_PosY_cor = new ReadWritePlot;
/////////////////////////////////////////////////////////////////////////////////////////////////



