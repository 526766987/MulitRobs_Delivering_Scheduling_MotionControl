#pragma once

#include <vector>

using namespace std;

extern vector<vector<int>> s2sEnemyPath; //节点对中间路径
extern vector<vector<int>> s2sEnemyPair;  //节点对{i,j}
extern vector<float> s2sEnemyPriorty; //节点对重要度
extern vector<int> s2sEnemyRank; //操作等级
extern vector<vector<float>> s2sEnemyDist;  //节点距离K*K
extern float maxEnemyDist; //最远两个节点的距离max(s2sDist)
extern bool reachEnemyStation[4][50]; // 能到达的站
extern bool attackerInPostion[4]; // 攻击者是否在攻击点就位
extern int attackerState[4]; //攻击者状态
extern float old_defensiveCoor[4][2]; //攻击目标坐标记录
extern int old_attackerState[4]; //攻击者状态记录
extern int unattackTimer[4]; //记录当前攻击点没有敌方来的持续帧数
extern int currentAttackEnemyStation[4]; //记录当前攻击站点编号

// 敌方站点捉对
void station_pair_attacker();
void flood_attacker(int rob_i);
void all_flood_attacker();

// 站点对的权重计算
void move_priorty_attacker();

// 敌方角落站
extern vector<int> cornerEnemyStation;
extern vector<int> cornerEnemyStationKind;
void corner_enemy_station();

// 敌方管道站
extern vector<int> pipeEnemyStation;
void pipe_enemy_station();

// 敌方关键节点
extern vector<int> keyEnemyStation;
void key_enemy_station_attacker();

// 攻击者选择器
void attacker_choose(int attackerAmount);

// 攻击目标实时选择器，返回是否执行成功
void attack_target_choose(int rob_i);

//攻击者跟随
void attacker_follow(int rob_i);

// 攻击者运动控制
void defensive_motion_control(int rob_i, float defensiveCoor[2]);

// 攻击者状态轮转
void attack_wheel();

//攻击者追尾行动
void crazy_attacker_follow(int rob_i);