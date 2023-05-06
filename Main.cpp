#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <numeric>
#include <functional>
#include "Global.h"
#include "Main.h"
#include "Readin.h"
#include "Map.h"
#include "Calculate.h"
#include "Schedule.h"
#include "Motion.h"
#include "Attack.h"
#include "Out.h"
#include "Path.h"
#include "Defend.h"

using namespace std;

/* 初始化（前5秒准备时间） */
void init()
{
    //参数初始化
    init_param();

    //地图类型识别与地图构建 // 【Map.cpp】
    wall_growth(true);
    chess_board_creat();
    outdebug("Map Create Done.");

    //站点配对（包含站点对权重计算） // 【Schedule.cpp】
    station_pair();
    outdebug("Station Pair Done.");

    //PID初始化 // 【Motion.cpp】
    for (int rob_i = 0; rob_i < 4; rob_i++) {
        rotate_PID_init(rob_i);
    }

    //功防关键节点的判断 // 【Attack.cpp】
    corner_enemy_station();
    pipe_enemy_station();
    corner_station();
    //敌方工作站配对
    station_pair_attacker();
    //攻击点搜索
    key_enemy_station_attacker();

    // 地图识别 // 【Map.cpp】
    map_identify();
    outdebug("mapKind", MAPKIND);

    //攻击者选取
    if (MAPKIND == 1)
    {
        attacker_choose(0);
    }
    else if (MAPKIND == 2)
    {
        attacker_choose(1);
    }
    else if (MAPKIND == 3)
    {
        attacker_choose(1);
    }
    else if (MAPKIND == 4)
    {
        if (state.K == 0)
        {
            attacker_choose(4);
        }
        else
        {
            attacker_choose(0);
        }
    }
    outdebug("Attacker:", state.robot[0].attacker, state.robot[1].attacker, state.robot[2].attacker, state.robot[3].attacker);
}

// 运行入口：主函数
int main() {

    readMapUntilOK();
    outdebug("Read Map DONE.");

    init();
    outdebug("INIT DONE.");

    puts("OK");
    fflush(stdout);

    int frameID;
    while (scanf("%d", &frameID) != EOF) {
        state.timeStamp = frameID;
        outdebug("--------第", state.timeStamp, "帧--------");
        readUntilOK();
        printf("%d\n", frameID);

        // 雷达索敌与危险站确定 // 【Defend.cpp】
        scouting();
        // 调度策略（调度状态轮转）//【Schedule.cpp】
        state_wheel();
        // 攻击策略（攻击者状态轮转） // 【Attacker.cpp】
        attack_wheel();
        // 实时路径规划 // 【Path.cpp】
        path_search();
        // 避障（安全点搜索与冲突避让决策） // 【Path.cpp】
        avoid();
        // 运动控制 // 
        move();

        //本帧处理完毕
        printf("OK\n");
        fflush(stdout);
    }
    outfile0.close();
    outfile1.close();
    return 0;
}
