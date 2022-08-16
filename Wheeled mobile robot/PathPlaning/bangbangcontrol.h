#ifndef BANGBANGCONTROL_H
#define BANGBANGCONTROL_H



const double a_max = 20000;//mm/s
const double v_max = 3000;
const double d_max = 20000; //mm/s
const double dt = 0.002; //s

enum Mode {
    RealTime,
    Table
};


double get_t_max(double x1, double v0, double v1, 
                 double a_max, double d_max, double v_max);

double compute_motion_1d(double x1, double v0, double v1,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time, double &traj_time_acc, double &traj_time_dec, double &traj_time_flat,
					   Mode mode,double dt);

void compute_motion_2d(double &x1, double &v0_x, double v1_x, 
                       double &y1, double &v0_y, double v1_y,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time_x, double &traj_time_acc_x, double &traj_time_dec_x, double &traj_time_flat_x,
                       double &traj_time_y, double &traj_time_acc_y, double &traj_time_dec_y, double &traj_time_flat_y,
					   Mode mode,double dt);

                       
                       
#endif


 
