#pragma once

// PID参数结构体
struct Rotate_PID_param
{
    float err_r;             		//定义偏差值
    float err_last_r;          		//定义上一个偏差值
    float Kp_r, Ki_r, Kd_r;         //定义空载状态下比例、积分、微分系数
    float integral_r;          		//定义积分值
    float v_r;             		    //定义速度值,亦即控制量
    float v_last_r;          		//定义上一个速度值
    float old_target[2];
};

// PID参数实例声明
extern Rotate_PID_param rotate_PID_param[4];

// 目标角度与当前航向夹角
extern float clampingAngle[4];

// PID参数初始化
void rotate_PID_init(int rob_i);

// 根据路径输出控制指令
void motion_by_path();

// 人工势场修正的运动控制core
void forward_rotate_control_APF(int rob_i, float Target_Coord[2]);

// 线速度控制器
float forward_control(int rob_i, float Target_Coord[2]);

// 角速度控制器
float rotate_control(int rob_i, float Target_Coord[2]);

// 运动控制(总)
void move();

// 冲撞策略
float forward_control_cash(int rob_i, float Target_Coord[2]);

void crash_motion_control(int rob_i, float defensiveCoor[2]);