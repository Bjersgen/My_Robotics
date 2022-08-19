#include <iostream>
#include <math.h>
#include <stdio.h>
#include "bangbangcontrol.h"
#include <fstream>
#include <vector>
#include <iomanip>
//#include <utils.h>
//#include "geometry.h"

using namespace std;


bool debug_flag = true; 
double a[2] = {0}; //ax,ay for realtime
double v[2] = {0}; //vx,vy for realtime
double delta_pos[2] = {0}; //dx,dy for realtime

int CaseX = 0;//debug
int CaseY = 0;

vector<double> accTable;
vector<double> velTable;
vector<double> posTable;

double compute_motion_1d(double x1, double v0, double v1,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time, double &traj_time_acc, double &traj_time_dec, double &traj_time_flat,
					   Mode mode,double dt,PlanType PT)
{
	

    if(mode == RealTime){            
		
		int dis_flag = 1;
		int vel_flag = 1;
		if(x1<0) dis_flag = -1;
		if(v1-v0<0) vel_flag = -1;
        //case1:已到点，迅速停下或已达到目标速度
		double a = dt;
        if(fabs(x1) <= v_max*dt) {
			if(fabs(v0) <= 0.5*a_max*dt) v0 = 0;
			if(PT == X)CaseX =1;
			if(PT == Y)CaseY =1;
			return (v0 ==0 )?0:vel_flag*d_max;
			}
        //case2:速度与位移同向，讨论
        if(x1*v0 >= 0 ){

            if(PT == X)CaseX = 2;
			if(PT == Y)CaseY = 2;
            double distance = fabs(x1);
            v0 = fabs((v0>v_max)?v_max:v0);
            double to_target_min_dis = fabs(((v1*v1)-(v0*v0))/(2*a_max));
            double min_acc_dis = fabs(((v_max*v_max)-(v0*v0))/(2*a_max));
            double min_dec_dis = fabs(((v_max*v_max)-(v1*v1))/(2*a_max));
			//距离够匀速
            if(min_acc_dis + min_dec_dis < distance ){
                if(v0 == v_max) return 0;
                else return a_max*dis_flag;
            } 
			
			//距离不够，直接根据末速度加减速
            if(to_target_min_dis >= distance){
				//std::cout<< PT <<endl;
				std::cout<<" to_target_min_dis: "<< to_target_min_dis<<std::endl;
				// std::cout<<" distance: "<< distance <<std::endl;
                // std::cout<<" can not achieve the position and velocity at the same time "<<std::endl;
                return a_max*vel_flag;
            }
			//距离够了,但不够匀速，根据位移加减速
            if(min_acc_dis + min_dec_dis >= distance && v0 != v_max){
                return a_max*dis_flag;
            }
            
           
            
        }
        //case3:速度与位移反向，先减速到0，然后进入同向讨论
        if((x1) * v0 < 0 ){
			if(PT == X)CaseX = 3;
			if(PT == Y)CaseY = 3;
            int vel_flag = -1;
            if(v0<0) vel_flag = 1;
            return d_max*vel_flag;
        }
    }

	else if (mode == Table)
	{

		double acc1_time_to_v1 = fabs(v1-v0)/a_max;
		double acc1_dist_to_v1 = fabs((v1+v0)/2.0) *acc1_time_to_v1;
		double dec1_time_to_v1 = fabs(v0-v1)/d_max;
		double dec1_dist_to_v1 = fabs((v1+v0)/2.0) *dec1_time_to_v1;
		double v_c = 0; //
		double t_max = 0; //v_max lasting time
		// (v_max+v0)(v_max-v0)/(2*a_max)+(v_max+v1)(v_max-v1)/(2*d_max)+v_max*t_max = x1
		double timecount = 0;
		if(v0*x1<0){
			double  t_d = v0/d_max;
			double  x_d = (v0*v0)/(2*d_max);
			v0 = fabs(v0);
			x1 = fabs(x1);
			while(v0 != 0){

			}
			compute_motion_1d((x1+x_d), v0, v1, a_max, d_max, v_max,
			traj_time, traj_time_acc,traj_time_dec,traj_time_flat,Table,dt,PT);
			
		}
		else{
			
			t_max = get_t_max(x1,v0,v1,a_max,d_max,v_max);
			if(t_max >= 0){
				
				traj_time_acc = (v_max-v0)/a_max;
				traj_time_dec = (v_max-v1)/d_max;
				traj_time_flat = t_max;
				traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
				
			}
			
			if(t_max < 0){
				
				if(fabs(v0)>fabs(v1) && dec1_dist_to_v1>fabs(x1)){
					
					traj_time_acc = 0;
					traj_time_dec = fabs(sqrt(v0*v0-2*x1*d_max) - v0)/d_max;
					traj_time_flat =0;
					traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
					
				}
				else if(fabs(v0)<fabs(v1) && acc1_dist_to_v1>fabs(x1)){
					
					traj_time_dec = 0;
					traj_time_acc = fabs(sqrt(v0*v0+2*x1*a_max) - v0)/a_max;
					traj_time_flat =0;
					traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
					
					
				}
				else{
					double v_top = 0;
					//(pow(v_top,2)-pow(v0,2))/(2*a_max)+(pow(v_top,2)-pow(v1,2))/(2*d_max) = x1;
					v_top = sqrt((2*a_max*d_max*x1+a_max*v1*v1+d_max*v0*v0)/(a_max+d_max));
					traj_time_dec = (v_top - v1)/d_max;
					traj_time_acc = (v_top - v0)/a_max;
					traj_time_flat =0;
					traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
					
				}
			}
		}
	}
    return 0;
}


void compute_motion_2d(double &x1, double &v0_x, double v1_x, 
                       double &y1, double &v0_y, double v1_y,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time_x, double &traj_time_acc_x, double &traj_time_dec_x, double &traj_time_flat_x,
                       double &traj_time_y, double &traj_time_acc_y, double &traj_time_dec_y, double &traj_time_flat_y,
					   Mode mode,double dt){
	
	if(mode == RealTime){

		a[0] = compute_motion_1d(x1, v0_x, v1_x, a_max, d_max, v_max,
				traj_time_x, traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,
				mode,dt,X);
		
	    a[1] = compute_motion_1d(y1, v0_y, v1_y, a_max, d_max, v_max,
				traj_time_y, traj_time_acc_y,traj_time_dec_y,traj_time_flat_y,
				mode,dt,Y);
        //new velocity and send to function motionplanner and for next time compute_motion_2d
		v[0] = v0_x + a[0]*dt;
        v[1] = v0_y + a[1]*dt;

		if(v[0]<-v_max) v[0] = -v_max;
		if(v[0]> v_max) v[0] =  v_max;
		if(v[1]<-v_max) v[1] = -v_max;
		if(v[1]> v_max) v[1] =  v_max;
		if(fabs(v[0])<0.5*a_max*dt) v[0] = 0;
		if(fabs(v[1])<0.5*a_max*dt) v[1] = 0;
		//new position for next time compute_motion_2d
		delta_pos[0] = (v[0]+v0_x)*dt/2;
		delta_pos[1] = (v[1]+v0_y)*dt/2; 
		
		v0_x = v[0];
        v0_y = v[1];

		x1 = x1 - delta_pos[0];
        y1 = y1 - delta_pos[1];
	}

	else if(mode == Table){

		compute_motion_1d(x1, v0_x, v1_x, a_max, d_max, v_max,
				traj_time_x, traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,
				mode,dt,X);
		
	    compute_motion_1d(y1, v0_y, v1_y, a_max, d_max, v_max,
				traj_time_y, traj_time_acc_y,traj_time_dec_y,traj_time_flat_y,
				mode,dt,Y);

	}

	
}

double get_t_max(double x1, double v0, double v1, 
                 double a_max, double d_max, double v_max){
					 
	double x_max = x1-(pow(v_max,2)-pow(v0,2))/(2*a_max)-(pow(v_max,2)-pow(v0,2))/(2*d_max);
	double t = x_max/v_max;
	return t;
	
}

//test
int main(int argc, char** argv)
{

	double traj_time_x = 0;
	double traj_time_acc_x = 0;
	double traj_time_dec_x = 0;
	double traj_time_flat_x = 0;
    double traj_time_y = 0;
	double traj_time_acc_y = 0;
	double traj_time_dec_y = 0;
	double traj_time_flat_y = 0;
	double x1 = -3000;
	double v0_x = 0;
	double v1_x = 0;
    double y1 = -3000; 
	double v0_y = 0;
	double v1_y = 0;
	ofstream out_txt_file;
	if(debug_flag){

		
		out_txt_file.open("/home/pi/Desktop/data2.txt",ios::out | ios::trunc);
		out_txt_file << fixed;

	}
	
    //当前控制频率下，position X以及position Y 的最小分辨率是
	while(fabs(x1)>v_max*dt | fabs(y1)>v_max*dt | fabs(v0_x)>a_max*dt | fabs(v0_y)>a_max*dt){

		compute_motion_2d(x1,v0_x,v1_x,y1,v0_y,v1_y,a_max,d_max,v_max,traj_time_x,traj_time_acc_x,traj_time_dec_x,
		traj_time_flat_x,traj_time_y,traj_time_acc_y,traj_time_dec_y,traj_time_flat_y,RealTime,dt);
		
		if(debug_flag){

			out_txt_file<< setprecision(0)<< a[0];
			out_txt_file<< '\t';
			out_txt_file<< setw(10)<< setprecision(0)<<a[1];
			out_txt_file<< setw(10)<< setprecision(1)<<v0_x;
			out_txt_file<< setw(10)<< setprecision(1)<<v0_y;
			out_txt_file<< setw(10)<< setprecision(2)<<x1;
			out_txt_file<< setw(10)<< setprecision(2)<<y1;
			out_txt_file<< setw(10)<< setprecision(0)<<CaseX;
			out_txt_file<< setw(10)<< setprecision(0)<<CaseY;
			out_txt_file<< '\n';
			// cout<< "ax:"<<a[0]<<endl;
			// cout<< "ay:"<<a[1]<<endl;
			// cout<< "vx:"<<v[0]<<endl;
			// cout<< "vy:"<<v[1]<<endl;
			// cout<< "x:"<<x1<<endl;
			// cout<< "y:"<<y1<<endl;

		}
		
	}
    if(debug_flag) out_txt_file<<endl;
	// cout<<"traj_time_x:"<<traj_time_x<<endl;
	// cout<<"traj_time_y:"<<traj_time_y<<endl;
	return 0 ;

}