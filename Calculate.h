#pragma once

/*--------------坐标变换-----------------*/

// 根据输入的第几列算中心坐标
float col2x(int col);

// 根据输入的第几行算中心坐标
float row2y(int row);

// 根据输入的坐标算第几列
int x2col(float x);

// 根据输入的坐标算第几行
int y2row(float y);

// 根据输入的第几列算中心坐标
int x2col_chess(float x);

// 根据输入的第几行算中心坐标
int y2row_chess(float y);

// 根据输入的坐标算第几列
float col2x_chess(int col);

// 根据输入的坐标算第几行
float row2y_chess(int row);

/*--------------距离计算-----------------*/

// 曼哈顿距离
float manhattan_distance(int p1_i, int p1_j, int p2_i, int p2_j);

// 计算机器人中心距离距离地图边缘的距离
float cal_map_distance_d(float robot_coor[2]);

// 沿当前朝向到地图边缘的距离
float cal_map_distance(int rob_i);

// 目标角度与当前航向夹角
float clamping_angle(int rob_i, float Target_Coord[2]);

// 新版综合路程计算
float new_dist_cal(float distr2b, float distb2s, float theta);

/*--------------其他计算-----------------*/

// 计算每种产品的全局价值：G × (1 - α^-k) × μ
float cal_productValue(int type);

// 用于计算{时间价值系数}和{碰撞价值系数}的函数f
float f_func(float x, float maxX, float minRate);

// 是否存在某产品
bool pro_exsit(int type, int materials);

// 综合收入计算
float income_cal(int takeS, int sellS);

// 新版综合收入计算
float new_income_cal(int takeS, int sellS, float s2spriorty);
