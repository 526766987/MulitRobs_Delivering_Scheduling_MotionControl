#pragma once

#include <vector>

using namespace std;

// 调度指令结构体
struct schedulingCommond
{
    int takeStation; //购入地
    int sellStation; //卖出地
    int state;  //执行状态，-1 ATTACK, 0初始态，1取货态，2运货态，3不可改取货态
    float efficient; //指令的效益
};

extern bool newIncomeSwitch; // 是否使用高级综合收益
extern vector<vector<int>> s2sPath; //节点对中间路径
extern vector<vector<int>> s2sPair;  //节点对{i,j}
extern vector<float> s2sPriorty; //节点对重要度
extern vector<int> s2sRank; //操作等级
extern int actionRank[5]; //操作等级映射（7，456->7，123->456,456,123）
extern vector<vector<float>> s2sDist;  //节点距离K*K
extern vector<vector<float>> s2sTimeLose; //理想时间衰减
extern float maxDist; //最远两个节点的距离max(s2sDist)
extern vector<vector<int>> criticalPair; // 关键路径节点对
extern vector<float> criticalPairPriorty; //关键路径节点对重要度
extern vector<int> criticalPairRank; //关键路径节点对操作等级

/*-------------- 机器人指令 -----------------*/

// 当前指令实例声明
extern schedulingCommond cmd[4];

// 上一次的历史指令实例声明
extern schedulingCommond cmdOld[4];

// 虚拟调度器输出指令实例声明
extern schedulingCommond cmdVis[4];

// 上一次的买卖站（仅在买卖发生时刷新，用作买卖失败后的重置保险） 0 take 1 sell
extern int lastOperationStation[4][2];

//一直在执行相同的任务，说明有死锁可能，尝试让他横冲直撞，这个功能有问题，暂时只计时不冲撞
extern int sameCmdTime[4];

/*--------------节点对确定-----------------*/

// 站点捉对
void station_pair();

// 站点对的权重计算
void move_priorty();

/*--------------贪婪调度器-----------------*/

// 实时贪婪调度器
bool real_time_greedy_scheduler(int rob_i, bool newIncomeSwitch, bool visualSwitch, int rob_preempt);

// 调度器核心
void scheduler_core();

// 在工作台上进行状态轮转与调度
void state_wheel();

// 实时闲置预跑动调度器
bool real_time_runaway_scheduler(int rob_i, bool chooseFinishStationSwitch);

/*--------------关键路径调度器-----------------*/

// 在确认sta_i站缺少mat_i材料后，寻找能供上的站点
void find_criticalPair(int sta_i, int mat_i);

// 查询站7->8/9的priority
float sell7_search(int sta_i);

// 关键路径搜索
void critical_taskpair();

// 测试旧指令是否存在冲突（能否还原旧指令）
bool test_old_cmd(int rob_i);

// 关键路径调度器
void critical_scheduler();

// 被堵卖出换点
void bad_seller();