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
#include "Map.h"
#include "Calculate.h"
#include "Schedule.h"
#include "Motion.h"
#include "Attack.h"
#include "Out.h"
#include "Path.h"
#include "Defend.h"

using namespace std;

// 部分参数初始计算
void init_param() {
    //outdebug("本图编号：", map_num);
    stationType[4].gain *= mapparams[MAPKIND].L2;
    stationType[5].gain *= mapparams[MAPKIND].L2;
    stationType[6].gain *= mapparams[MAPKIND].L2;
    stationType[7].gain *= mapparams[MAPKIND].L3;

    //计算价值
    for (int i = 1; i <= 7; i++) {
        stationType[i].productValue = cal_productValue(i);
    }
}

/* 读取初始化地图信息 */
bool readMapUntilOK() {
    char line[1024];
    int row = 0;
    int robot_index = 0;
    int station_index = 0;
    int enemy_station_index = 0;

    //队伍识别
    fgets(line, sizeof line, stdin);
    if (line[0] == 'B') {
        TEAM = 0;
    }
    else if (line[0] == 'R') {
        TEAM = 1;
    }

    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //开始处理第row行地图信息存入state结构体变量
        for (int i = 0; i < 100; i++) {
            //墙壁
            if (line[i] == '#') {
                wallMap[i][row] = true;
                continue;
            }
            //己方为蓝方
            if (TEAM == 0) {
                if (line[i] == 'A') {
                    state.robot[robot_index].p_ij_init[0] = i;
                    state.robot[robot_index].p_ij_init[1] = row;
                    state.robot[robot_index].p[0] = col2x(i);
                    state.robot[robot_index].p[1] = row2y(row);
                    outdebug(robot_index, "号机器人坐标:", state.robot[robot_index].p[0], state.robot[robot_index].p[1]);
                    robot_index++;

                }
                else if (line[i] >= '1' && line[i] <= '9') {
                    state.station[station_index].p_ij[0] = i;
                    state.station[station_index].p_ij[1] = row;
                    state.station[station_index].p[0] = col2x(i);
                    state.station[station_index].p[1] = row2y(row);
                    state.station[station_index].type = line[i] - '0';
                    state.K += 1;
                    stationType[state.station[station_index].type].quantity += 1;
                    outdebug(station_index, "号工作台坐标:", state.station[station_index].p[0], state.station[station_index].p[1], state.station[station_index].p_ij[0], state.station[station_index].p_ij[1]);
                    station_index++;
                }
                else if (line[i] >= 'a' && line[i] <= 'i') {
                    state.enemyStation[enemy_station_index].p_ij[0] = i;
                    state.enemyStation[enemy_station_index].p_ij[1] = row;
                    state.enemyStation[enemy_station_index].p[0] = col2x(i);
                    state.enemyStation[enemy_station_index].p[1] = row2y(row);
                    state.enemyStation[enemy_station_index].type = line[i] - 96;
                    state.eK += 1;
                    outdebug(enemy_station_index, "号敌方工作台坐标:", state.enemyStation[enemy_station_index].p[0], state.enemyStation[enemy_station_index].p[1], state.enemyStation[enemy_station_index].p_ij[0], state.enemyStation[enemy_station_index].p_ij[1]);
                    enemy_station_index++;
                }
                wallMap[i][row] = false;
            }
            //己方为红方
            else if (TEAM == 1) {
                if (line[i] == 'B') {
                    state.robot[robot_index].p_ij_init[0] = i;
                    state.robot[robot_index].p_ij_init[1] = row;
                    state.robot[robot_index].p[0] = col2x(i);
                    state.robot[robot_index].p[1] = row2y(row);
                    outdebug(robot_index, "号机器人坐标:", state.robot[robot_index].p[0], state.robot[robot_index].p[1]);
                    robot_index++;

                }
                else if (line[i] >= 'a' && line[i] <= 'i') {
                    state.station[station_index].p_ij[0] = i;
                    state.station[station_index].p_ij[1] = row;
                    state.station[station_index].p[0] = col2x(i);
                    state.station[station_index].p[1] = row2y(row);
                    state.station[station_index].type = line[i] - 96;
                    state.K += 1;
                    stationType[state.station[station_index].type].quantity += 1;
                    outdebug(station_index, "号工作台坐标:", state.station[station_index].p[0], state.station[station_index].p[1], state.station[station_index].p_ij[0], state.station[station_index].p_ij[1]);
                    station_index++;
                }
                else if (line[i] >= '1' && line[i] <= '9') {
                    state.enemyStation[enemy_station_index].p_ij[0] = i;
                    state.enemyStation[enemy_station_index].p_ij[1] = row;
                    state.enemyStation[enemy_station_index].p[0] = col2x(i);
                    state.enemyStation[enemy_station_index].p[1] = row2y(row);
                    state.enemyStation[enemy_station_index].type = line[i] - '0';
                    state.eK += 1;
                    outdebug(enemy_station_index, "号敌方工作台坐标:", state.enemyStation[enemy_station_index].p[0], state.enemyStation[enemy_station_index].p[1], state.enemyStation[enemy_station_index].p_ij[0], state.enemyStation[enemy_station_index].p_ij[1]);
                    enemy_station_index++;
                }
                wallMap[i][row] = false;
            }
        }
        row++;
    }
    return false;
}

/* 读取帧信息 */
bool readUntilOK() {
    char line[8848];
    int lineCount = 0;
    int K = state.K;
    //outdebug("开始读取帧数据");
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        if (lineCount == 0)
        {
            //第一行输入 2 个整数，表示帧序号（从 1 开始递增）、当前金钱数。
            sscanf(line, "%d", &state.cash);
            //outdebug("钱钱：", state.cash);
        }
        else if (lineCount == 1)
        {
            //第二行输入 1 个整数，表示场上工作台的数量 K（K <= 50）。
            int err = sscanf(line, "%d", &state.K);
            K = state.K;
            //outdebug(state.K);
        }
        else if (lineCount < 2 + K)
        {
            //接着 K 行数据，每一行表示一个工作台，分别由如下所示的数据构成，共计 6 个数字：
            int temp;
            int err = sscanf(line, "%d %f %f %d %d %d",
                &state.station[lineCount - 2].type,
                &state.station[lineCount - 2].p[0],
                &state.station[lineCount - 2].p[1],
                &state.station[lineCount - 2].inBusy,
                &state.station[lineCount - 2].materials,
                &temp);
            state.station[lineCount - 2].finish = (temp == 1);
        }
        else if (lineCount < 2 + K + 4)
        {
            //接下来的 4 行数据，每一行表示一个机器人，分别由如下表格中所示的数据构成，每行 10 个数字。
            int err = sscanf(line, "%d %d %f %f %f %f %f %f %f %f",
                &state.robot[lineCount - 2 - K].atStation,
                &state.robot[lineCount - 2 - K].bring,
                &state.robot[lineCount - 2 - K].timeValue,
                &state.robot[lineCount - 2 - K].collisionValue,
                &state.robot[lineCount - 2 - K].angularSpeed,
                &state.robot[lineCount - 2 - K].speed[0],
                &state.robot[lineCount - 2 - K].speed[1],
                &state.robot[lineCount - 2 - K].direction,
                &state.robot[lineCount - 2 - K].p[0],
                &state.robot[lineCount - 2 - K].p[1]
            );
            //outdebug("第", lineCount - 2 - K, "号机器人附近的工作台：", state.robot[lineCount - 2 - K].atStation);
        }
        //雷达激光读取
        else if (lineCount < 2 + K + 4 + 4)
        {
            //再 4 行数据，每一行表示一个机器人的360个激光。
            char* token = strtok(line, " ");
            int i = 0;
            int maxLaser_i = 0;
            int minLaser_i = 0;
            state.robot[lineCount - 2 - K - 4].sumLaser = 0;
            while (token != NULL) {
                //读取原始记录
                state.robot[lineCount - 2 - K - 4].lasers[i] = stof(token);
                //state.robot[lineCount - 2 - K - 4].lasers[i] = state.robot[lineCount - 2 - K - 4].lasers[i] < 5 ? state.robot[lineCount - 2 - K - 4].lasers[i] : 5;
                //更新极值索引
                maxLaser_i = state.robot[lineCount - 2 - K - 4].lasers[i] > state.robot[lineCount - 2 - K - 4].lasers[maxLaser_i] ? i : maxLaser_i;
                minLaser_i = state.robot[lineCount - 2 - K - 4].lasers[i] < state.robot[lineCount - 2 - K - 4].lasers[minLaser_i] ? i : minLaser_i;
                // 求激光距离和（用于衡量空间宽裕度）
                //state.robot[lineCount - 2 - K - 4].sumLaser += (state.robot[lineCount - 2 - K - 4].lasers[i] < robBlockDistance[map_num]) ? robBlockDistance[map_num] : state.robot[lineCount - 2 - K - 4].lasers[i];
                state.robot[lineCount - 2 - K - 4].sumLaser += state.robot[lineCount - 2 - K - 4].lasers[i];
                //计算相对于地图原点的角度 
                float mapdirection = state.robot[lineCount - 2 - K - 4].direction + (static_cast<double>(i) / 360) * 2 * PI;
                if ((mapdirection < -PI) || (mapdirection > PI)) {
                    mapdirection = mapdirection - 2 * PI;
                }
                state.robot[lineCount - 2 - K - 4].lasersMapDirection[i] = mapdirection;
                //计算激光落点及对应格子
                float x = state.robot[lineCount - 2 - K - 4].p[0] + 1.001 * state.robot[lineCount - 2 - K - 4].lasers[i] * cos(mapdirection);
                float y = state.robot[lineCount - 2 - K - 4].p[1] + 1.001 * state.robot[lineCount - 2 - K - 4].lasers[i] * sin(mapdirection);
                //检测该点是否为墙壁,不是墙壁则为机器人
                if (!wallMap[x2col(x)][y2row(y)]) {
                    state.robot[lineCount - 2 - K - 4].lasersObject[i] = 1;
                }

                //更新迭代器
                token = strtok(NULL, " ");
                i++;
                //outdebug("laser","[",i,"]=",state.robot[lineCount - 2 - K - 4].lasers[i]);
                //outfile << "laser[" << i << "] = " << state.robot[lineCount - 2 - K - 4].lasers[i] << endl;
            }
            state.robot[lineCount - 2 - K - 4].maxLaser_i = maxLaser_i;
            state.robot[lineCount - 2 - K - 4].minLaser_i = minLaser_i;
        }
        lineCount++;
    }
    return false;
}
