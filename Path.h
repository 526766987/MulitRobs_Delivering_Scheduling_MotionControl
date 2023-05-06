#pragma once

#include <vector>

using namespace std;

// 能到达的站
extern bool reachStation[2][4][50];
// 能到达的机器人
extern bool reachRobot[2][4][4];
// 路径
extern vector<vector<int>> path;
// 机器人挡住的格子
extern int robotBlockPosition[2][4][13];
extern bool pathMap[4][100][100];
extern bool pathChess[4][101][101];

/*--------------动态障碍-----------------*/

// 统计机器人挡住的格子
void robot_block_init();

// 判断位置是否被机器人占据
bool have_block(int rob_i, int p_i, int p_j, bool mapSwitch);

/*--------------寻路算法-----------------*/

// 从机器人rob_i出发泛洪
void flood(int rob_i, bool mapSwitch);

// 机器人依序泛洪（被其它机器人触及过的机器人不需要再泛洪，直接复制）
void all_flood(bool mapSwitch);

// A star 自动寻路
bool a_star_search(int start, int goal, bool mapSwitch, bool robotBlockSwitch, int rob_i, bool enemyBlockSwitch);

/*--------------路径规划-----------------*/

// 判断任意两点之间有没有墙
bool through_wall(int start, int goal, float stepLength, bool mapSwitch, bool robotBlockSwitch, int rob_i, bool enemyBlockSwitch);

// 去除路径中的冗余点
void path_simple(int rob_i, bool mapSwitch, bool robotBlockSwitch, bool enemyBlockSwitch);

// 实时路径规划
void real_time_path_search(int rob_i, bool robotBlockSwitch);

// 四个依序规划
void real_time_path_search_for_all(bool robotBlockSwitch);

/*--------------阻塞关系-----------------*/

extern bool robotBlockByRobot[4][4];
extern float robotBlockByRobotDist[4][4];
extern int nearestRobot[4];
extern float nearestRobotDist[4];
extern int robotAviod[4];
extern int safePlace[4];
extern float safePlaceDis[4];
extern bool inLeave[4];
extern bool inSlow[4];

// 验证路径是否被某机器人阻挡
void path_block_by_robot(int rob_i, bool mapSwitch, float stepLength);

// 寻找某机器人的就近安全点
int safe_place(int rob_i, bool mapSwitch);

// 重置信息
void init_avoid();

// 更新全部机器人占位
void path_block_by_robot_all();

// 寻找全部安全点
void safe_place_all();

// 根据阻塞关系确定回避的机器人
void avoid_choose();

// 躲避路径规划
void avoid_path_search();

// 路径搜索（总）
void path_search();

// 避障（安全点搜索与冲突避让决策）
void avoid();