#include "main.h"
#include "robot_sntestCM4.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <bangbang.h>
#include <utils.h>
#include <fstream>
#include <geometry.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @fn	double compute_motion_1d(double x0, double x1, double v0, double v1, double a_max, double d_max, double v_max, Mode mode)
///
/// @brief	   实时规划：一维状态下计算运动过程，输入当前速度和末速度，当前位置和末位置，最大加速度和最大速度，返回当前时刻加速度
///            保存为Table查询：查询速度以及加速度
///
/// @author	    Joker
/// @date		2022-8-2
///
/// @param	x0				初位置
/// @param  x1              末位置
/// @param	v0				初速度
/// @param	v1				末速度
/// @param	a_max			最大加速度
/// @param	d_max		    最大减速度
/// @param	v_max			最大速度
/// @param	mode			规划方式
////////////////////////////////////////////////////////////////////////////////////////////////////


//非零速到点,实时规划，只需要返回加速度a_max/-a_max/0/d_max/-d_max,速度通过其他函数积分
double compute_motion_1d(double x0, double x1, double v0, double v1, double a_max, double d_max, double v_max, Mode mode, planType PT){

    if(mode == RealTime){
        //case1:已到点，迅速停下
        if(x1-x0 == 0) return (v0==0)?0:d_max;
        //case2:速度与位移同向，讨论
        if((x1-x0) *v0 >= 0 ){

            int dis_flag = 1;
            int vel_flag = 1;
            if(x1-x0<0) dis_flag = -1;
            if(v1-v0<0) vel_flag = -1;

            double distance = fabs(x1 - x0);
            v0 = fabs((v0>v_max)?v_max:v0);
            double to_target_min_dis = fabs((v1*v1)-(v0*v0)/(2*a_max));
            double min_acc_dis = fabs((v_max*v_max)-(v0*v0)/(2*a_max));
            double min_dec_dis = fabs((v_max*v_max)-(v1*v1)/(2*a_max));

            //距离不够，直接根据末速度加减速
            if(to_target_min_dis > distance){
                std::cout<<" can not achieve the position and velocity at the same time "<<std::endl;
                return (vel_flag==1)?a_max:d_max;
            }
            //距离够了,但不够匀速，根据位移加减速
            if(min_acc_dis + min_dec_dis >= distance){
                return a_max*dis_flag;
            }
            //距离够匀速
            if(min_acc_dis + min_dec_dis < distance ){
                if(v0 == v_max) return 0;
                else return a_max*dis_flag;
            }
        }
        //case3:速度与位移反向，先减速到0，然后进入同向讨论
        if((x1-x0) * v0 < 0 ){
            int vel_flag = 1;
            if(v0<0) vel_flag = -1;
            return d_max*vel_flag;
        }
    }
    else if (mode == Table)
    {
        
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
/// @fn	CVector compute_motion_2d(CVector x0, CVector x1, CVector v0, CVector v1, double a_max, double d_max, double v_max, Mode mode)；
/// @brief 在二维下计算运动过程，给定初始速度向量和最终速度向量，以及最大加速度，最大速度
///
/// @author	
/// @date	2022-8-2
///
/// @param	x0						初位置
/// @param	v0						末位置
/// @param	v0						初速度
/// @param	v1						末速度
/// @param	a_max					最大加速度
/// @param	v_max					最大速度
/// @param	d_max				    最大减速度
////////////////////////////////////////////////////////////////////////////////////////////////////

CVector compute_motion_2d(CVector x0, CVector x1, CVector v0, CVector v1, double a_max, double d_max, double v_max, Mode mode) {

    if(mode == RealTime){
        CVector a ;
        a.x() = compute_motion_1d(x0.x(),x1.x(),v0.x(),v1.x(),a_max,d_max,v_max,mode);
        a.y() = compute_motion_1d(x0.y(),x1.y(),v0.y(),v1.y(),a_max,d_max,v_max,mode);
        return a;
    }


}
////////////////////////////////////////////////////////////////////////////////////////////////////
/// @fn	void compute_velocity(CVector x0, CVector x1, CVector v0, CVector v1, double a_max, double d_max, double v_max, Mode mode, double dt);
/// @brief 计算发给motionplanner的速度
///
/// @author	
/// @date	2022-8-2
///
/// @param	x0						初位置
/// @param	v0						末位置
/// @param	v0						初速度
/// @param	v1						末速度
/// @param	a_max					最大加速度
/// @param	v_max					最大速度
/// @param	d_max				    最大减速度
/// @param	dt				        周期
////////////////////////////////////////////////////////////////////////////////////////////////////

void compute_velocity(CVector x0, CVector x1, CVector v0, CVector v1, double a_max, double d_max, double v_max, Mode mode, double dt){

    if(mode == RealTime){

        CVector acc = compute_motion_2d(x0,x1,v0,v1,a_max,d_max,v_max,mode);
        double Vx = v0.x() + a.x()*dt;
        double Vy = v0.y() + a.y()*dt;    
        v0.x() = Vx;
        V0.y() = Vy;

    }
}

void control_init(){

    std::thread receiveThread(Localplanner);
    receiveThread.detach();

}

void Localplanner(){

    while(1){
        
        compute_velocity(x0,x1,v0,v1,a_max,d_max,v_max,RealTime,0.001);
        
    }


}