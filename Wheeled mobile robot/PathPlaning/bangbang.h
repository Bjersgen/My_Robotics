#ifndef BANGBANG_H
#define BANGBANG_H
//inline bool finite(double num)
//{
//	return fabs(num)<9999;
//}

//每次解完包立刻重新规划bangbangcontrol;
//bangbangcontrol规划单独一个线程，每次规划修改全局变量current_acc 供主线程主循环(2ms)的motionplanner()读取;
//每次解包后获取新的x0，未获取x0时，在x0的基础上积分(71Hz)
//a_max和d_max需要测

enum Mode {
    RealTime,
    Table
};

enum planType {
    MOVE_X,
    MOVE_Y,
    ROTATE
};

double compute_motion_1d(double x0, double x1, double v0, double v1, double a_max, double d_max, double v_max, Mode mode, planType PT);
CVector compute_motion_2d(CVector x0, CVector x1, CVector v0, CVector v1, double a_max, double d_max, double v_max, Mode mode);
void compute_velocity(CVector x0, CVector x1, CVector v0, CVector v1, double a_max, double d_max, double v_max, Mode mode, double dt);
/*
void compute_motion_1d(double x0, double v0, double v1,
                       double a_max, double d_max, double v_max, double a_factor, double vel_factor,
                       double &traj_accel, double &traj_time, double &traj_time_acc, double &traj_time_dec, 
                       double &traj_time_flat, planType pT, nonZeroMode mode = FAST);
void compute_motion_2d(CVector x0, CVector v0, CVector v1,
					   double a_max, double d_max, double v_max, double a_factor,
                       CVector &traj_accel, double &time, double &time_acc, double &time_dec, double &time_flat, nonZeroMode mode = FAST);
double compute_stop(double v, double max_a);*/


#endif
