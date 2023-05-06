#include <vector>
#include <cmath>
#include <functional>
#include <algorithm>
#include "Attack.h"
#include "Global.h"
#include "Map.h"
#include "Calculate.h"
#include "Path.h"
#include "Schedule.h"
#include "Out.h"
#include "Motion.h"

using namespace std;

// 能到达的站
bool reachEnemyStation[4][50] = { false };
vector<vector<int>> s2sEnemyPath; //节点对中间路径
vector<vector<int>> s2sEnemyPair;  //节点对{i,j}
vector<float> s2sEnemyPriorty; //节点对重要度
vector<int> s2sEnemyRank; //操作等级
vector<vector<float>> s2sEnemyDist(50, vector<float>(50));  //节点距离K*K
float maxEnemyDist; //最远两个节点的距离max(s2sDist)

bool attackerInPostion[4] = { false };
int attackerState[4] = { 0 };
float old_defensiveCoor[4][2] = { 0 };
int old_attackerState[4] = { 0 };


/* 角集 */
vector<int> cornerEnemyStation = {};
void corner_enemy_station()
{
    if (state.eK == 0)
    {
        return;
    }
    for (int emySta_i = 0; emySta_i < state.eK; emySta_i++)
    {
        int p_i = state.enemyStation[emySta_i].p_ij[0];
        int p_j = state.enemyStation[emySta_i].p_ij[1];

        //if ((p_i + 2 < 100) && (p_j + 2 < 100) && (wallMap[p_i + 2][p_j + 0] + wallMap[p_i + 0][p_j + 2] == 2))
        //{
        //    cornerEnemyStation.push_back(emySta_i);
        //}
        //else if ((p_i - 2 >= 0) && (p_j + 2 < 100) && (wallMap[p_i - 2][p_j + 0] + wallMap[p_i + 0][p_j + 2] == 2))
        //{
        //    cornerEnemyStation.push_back(emySta_i);
        //}
        //else if ((p_i - 2 >= 0) && (p_j - 2 >= 0) && (wallMap[p_i - 2][p_j + 0] + wallMap[p_i + 0][p_j - 2] == 2))
        //{
        //    cornerEnemyStation.push_back(emySta_i);
        //}
        //else if ((p_i + 2 < 100) && (p_j - 2 >= 0) && (wallMap[p_i + 2][p_j + 0] + wallMap[p_i + 0][p_j - 2] == 2))
        //{
        //    cornerEnemyStation.push_back(emySta_i);
        //}
        
        if (((p_i + 2 < 100) && wallMap[p_i + 2][p_j + 0]) + ((p_j + 2 < 100) && wallMap[p_i + 0][p_j + 2]) + (p_i + 2 >= 100) + (p_j + 2 >= 100) == 2)
        {
            cornerEnemyStation.push_back(emySta_i);
            cornerEnemyStationKind.push_back(0);//右下是墙角
        }
        else if (((p_i - 2 >= 0) && wallMap[p_i - 2][p_j + 0]) + ((p_j + 2 < 100) && wallMap[p_i + 0][p_j + 2]) + (p_i - 2 < 0) + (p_j + 2 >= 100) == 2)
        {
            cornerEnemyStation.push_back(emySta_i);
            cornerEnemyStationKind.push_back(1);//左下是墙角
        }
        else if (((p_i - 2 >= 0) && wallMap[p_i - 2][p_j + 0]) + ((p_j - 2 >= 0) && wallMap[p_i + 0][p_j - 2]) + (p_i - 2 < 0) + (p_j - 2 < 0) == 2)
        {
            cornerEnemyStation.push_back(emySta_i);
            cornerEnemyStationKind.push_back(2);//左上是墙角
        }
        else if (((p_i + 2 < 100) && wallMap[p_i + 2][p_j + 0]) + ((p_j - 2 >= 0) && wallMap[p_i + 0][p_j - 2]) + (p_i + 2 >= 100) + (p_j - 2 < 0) == 2)
        {
            cornerEnemyStation.push_back(emySta_i);
            cornerEnemyStationKind.push_back(3);//右上是墙角
        }
    }
}


/* 通道集 */
vector<int> pipeEnemyStation = {};
void pipe_enemy_station()
{
    if (state.eK == 0)
    {
        return;
    }
    for (int emySta_i = 0; emySta_i < state.eK; emySta_i++)
    {
        int p_i = state.enemyStation[emySta_i].p_ij[0];
        int p_j = state.enemyStation[emySta_i].p_ij[1];
        if ((p_i + 2 < 100) && (p_i - 2 >= 0) && (wallMap[p_i + 2][p_j + 0] + wallMap[p_i - 2][p_j + 0] == 2))
        {
            pipeEnemyStation.push_back(emySta_i);
        }
        else if ((p_j - 2 >= 0) && (p_j + 2 < 100) && (wallMap[p_i + 0][p_j - 2] + wallMap[p_i + 0][p_j + 2] == 2))
        {
            pipeEnemyStation.push_back(emySta_i);
        }
    }
}


/* 站点对的权重计算 */
vector<int> keyEnemyStation = {};
vector<int> cornerEnemyStationKind{};
void key_enemy_station_attacker()
{
    if (state.eK == 0)
    {
        return;
    }

    vector<int> critical_rank_list = { 5,4,3,2,1 };
    //actionRank[0] = 5; // 7
    //actionRank[1] = 4; // 456->7
    //actionRank[2] = 3; // 123->456
    //actionRank[3] = 2; // 456
    //actionRank[4] = 1; // 123

    //角站地形杀
    bool foundFlag = true;
    while (foundFlag)
    {
        foundFlag = false;
        for (int rank_k = 0; rank_k < 5; rank_k++)
        {
            float highestPriorty = -1;
            int findPair = -1;
            for (int pair_i = 0; pair_i < s2sEnemyPair.size(); pair_i++) {
                bool tempFlag = true;
                for (int cornSta_i = 0; cornSta_i < cornerEnemyStation.size(); cornSta_i++)
                {
                    if (cornerEnemyStation[cornSta_i] == s2sEnemyPair[pair_i][0])
                    {
                        //是角站
                        tempFlag = false;
                        break;
                    }
                }
                if (tempFlag) { continue; }

                if (s2sEnemyRank[pair_i] == critical_rank_list[rank_k])
                {
                    if (highestPriorty == -1)
                    {
                        highestPriorty = s2sEnemyPriorty[pair_i];
                        findPair = pair_i;
                    }
                    else if (s2sEnemyPriorty[pair_i] > highestPriorty)
                    {
                        highestPriorty = s2sEnemyPriorty[pair_i];
                        findPair = pair_i;
                    }
                }
            }

            if (findPair != -1)
            {
                bool tempFlag = true;
                for (int i = 0; i < keyEnemyStation.size(); i++)
                {
                    if (s2sEnemyPair[findPair][0] == keyEnemyStation[i])
                    {
                        tempFlag = false;
                        break;
                    }
                }
                if (tempFlag) { keyEnemyStation.push_back(s2sEnemyPair[findPair][0]); }

                s2sEnemyPath.erase(s2sEnemyPath.begin() + findPair);
                s2sEnemyPair.erase(s2sEnemyPair.begin() + findPair);
                s2sEnemyPriorty.erase(s2sEnemyPriorty.begin() + findPair);
                s2sEnemyRank.erase(s2sEnemyRank.begin() + findPair);
                foundFlag = true;
                break;
            }
        }
    }

    int cornerStaSize = keyEnemyStation.size();

    //其它
    while (s2sEnemyPair.size() > 0)
    {
        for (int rank_k = 0; rank_k < 5; rank_k++)
        {
            float highestPriorty = -1;
            int findPair = -1;
            for (int pair_i = 0; pair_i < s2sEnemyPair.size(); pair_i++) {
                if (s2sEnemyRank[pair_i] == critical_rank_list[rank_k])
                {
                    if (highestPriorty == -1)
                    {
                        highestPriorty = s2sEnemyPriorty[pair_i];
                        findPair = pair_i;
                    }
                    else if (s2sEnemyPriorty[pair_i] > highestPriorty)
                    {
                        highestPriorty = s2sEnemyPriorty[pair_i];
                        findPair = pair_i;
                    }
                }
            }

            if (findPair != -1)
            {
                bool tempFlag = true;
                for (int i = 0; i < keyEnemyStation.size(); i++)
                {
                    if (s2sEnemyPair[findPair][0] == keyEnemyStation[i])
                    {
                        tempFlag = false;
                        break;
                    }
                }
                if (tempFlag) { keyEnemyStation.push_back(s2sEnemyPair[findPair][0]); }

                s2sEnemyPath.erase(s2sEnemyPath.begin() + findPair);
                s2sEnemyPair.erase(s2sEnemyPair.begin() + findPair);
                s2sEnemyPriorty.erase(s2sEnemyPriorty.begin() + findPair);
                s2sEnemyRank.erase(s2sEnemyRank.begin() + findPair);
                break;
            }
        }
    }


    //9
    bool insertFlag = false;
    if ((state.enemyStation[keyEnemyStation[cornerStaSize]].type != 7))
    {
        insertFlag = true;
        int count = 0;
        for (int emySta_i = 0; emySta_i < state.eK; emySta_i++)
        {
            if (state.enemyStation[emySta_i].type == 9)
            {
                keyEnemyStation.insert(keyEnemyStation.begin() + cornerStaSize + count, emySta_i);
                count++;
            }
        }
    }
    for (int keyESta_i = cornerStaSize; keyESta_i < keyEnemyStation.size() - 1; keyESta_i++)
    {
        if ((state.enemyStation[keyEnemyStation[keyESta_i]].type == 7) && (state.enemyStation[keyEnemyStation[keyESta_i + 1]].type != 7))
        {
            insertFlag = true;
            int count = 0;
            for (int emySta_i = 0; emySta_i < state.eK; emySta_i++)
            {
                if (state.enemyStation[emySta_i].type == 9)
                {
                    keyEnemyStation.insert(keyEnemyStation.begin() + keyESta_i + 1 + count, emySta_i);
                    count++;
                }
            }
        }
    }
    if (insertFlag == false)
    {
        // 9
        for (int emySta_i = 0; emySta_i < state.eK; emySta_i++)
        {
            if (state.enemyStation[emySta_i].type == 9)
            {
                keyEnemyStation.push_back(emySta_i);
            }
        }
    }
}



/* 从机器人rob_i出发泛洪 */
// attacker目前默认走chessBoard
void flood_attacker(int rob_i)
{
    // 输入：
    // rob_i 当前机器人
    // mapSwitch == 0，扩展地图expandWall(100*100)，mapSwitch == 1，棋盘地图chessBoard(101*101)

    int mapSize = 101; //地图尺寸

    vector<int> frontier; //本轮探索方块
    vector<vector<bool>> haveDetect(mapSize, vector<bool>(mapSize, false)); // 记录是否被探测过

    int start = x2col_chess(state.robot[rob_i].p[0]) + y2row_chess(state.robot[rob_i].p[1]) * mapSize;

    frontier.push_back(start);

    while (frontier.size() > 0)
    {
        //取一个块，如果没有被探测过那么探测一遍，如果探测过了那就直接删掉、跳过
        int current, p_i, p_j;
        current = frontier[0];
        p_i = current % mapSize;
        p_j = current / mapSize;
        if (haveDetect[p_i][p_j] == true)
        {
            //从队列里删除当前块
            frontier.erase(frontier.begin());
            continue;
        }
        haveDetect[p_i][p_j] = true;
        // 如果当前块是某个机器人或者某个站点就修改reachStation和reachRobot
        for (int i = 0; i < state.eK; i++)
        {
            if ((state.enemyStation[i].p_ij[0] == p_i) && (state.enemyStation[i].p_ij[1] == p_j))
            {
                reachEnemyStation[rob_i][i] = true;
            }
        }
        //把周围的点加进来
        vector<vector<int>> neighbors;
        neighbors.push_back({ p_i - 1,p_j });
        neighbors.push_back({ p_i + 1,p_j });
        neighbors.push_back({ p_i ,p_j + 1 });
        neighbors.push_back({ p_i ,p_j - 1 });
        for (int nei_i = 0; nei_i < 4; nei_i++)
        {
            if ((neighbors[nei_i][0] >= 0) && (neighbors[nei_i][0] < mapSize) && (neighbors[nei_i][1] >= 0) && (neighbors[nei_i][1] < mapSize))//非边界
            {
                if (haveDetect[neighbors[nei_i][0]][neighbors[nei_i][1]] == false)//未探测过
                {
                    if (!(chessBoard[neighbors[nei_i][0]][neighbors[nei_i][1]])) //非墙
                    {
                        //把这个邻居写入队列
                        frontier.push_back(neighbors[nei_i][0] + neighbors[nei_i][1] * mapSize);
                    }
                }
            }
        }
        //从队列里删除当前块
        frontier.erase(frontier.begin());
    }
}


/* 机器人依序泛洪（被其它机器人触及过的机器人不需要再泛洪，直接复制）*/
void all_flood_attacker()
{
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        flood_attacker(rob_i);
    }
}

/* 站点对的权重计算 */
void move_priorty_attacker()
{
    int sta_i, sta_j;
    for (int pair_i = 0; pair_i < s2sEnemyPair.size(); pair_i++)
    {
        sta_i = s2sEnemyPair[pair_i][0];
        sta_j = s2sEnemyPair[pair_i][1];
        // 卖出7
        if (state.enemyStation[sta_i].type == 7)
        {
            s2sEnemyPriorty[pair_i] = 1 * (1 - s2sEnemyDist[sta_i][sta_j] / maxEnemyDist);
            s2sEnemyRank[pair_i] = actionRank[0];
        }
    }
    for (int pair_i = 0; pair_i < s2sEnemyPair.size(); pair_i++)
    {
        sta_i = s2sEnemyPair[pair_i][0];
        sta_j = s2sEnemyPair[pair_i][1];
        // 卖出456
        if ((state.enemyStation[sta_i].type <= 6) && (state.enemyStation[sta_i].type >= 4))
        {
            float upRank = -1;
            for (int pair_j = 0; pair_j < s2sEnemyPair.size(); pair_j++)
            {
                if (s2sEnemyPair[pair_j][0] == sta_j)
                {
                    upRank = (upRank < s2sEnemyPriorty[pair_j]) ? s2sEnemyPriorty[pair_j] : upRank;
                }
            }
            if (upRank == -1)
            {
                s2sEnemyPriorty[pair_i] = 1 * (1 - s2sEnemyDist[sta_i][sta_j] / maxEnemyDist);
                s2sEnemyRank[pair_i] = actionRank[3];
            }
            else
            {
                s2sEnemyPriorty[pair_i] = upRank * (1 - s2sEnemyDist[sta_i][sta_j] / maxEnemyDist);
                s2sEnemyRank[pair_i] = actionRank[1];
            }
        }
    }
}

/* 站点捉对 */
void station_pair_attacker()
{
    if (state.eK == 0)
    {
        return;
    }
    // 地图泛洪
    all_flood_attacker();

    maxEnemyDist = 0;
    for (int i = 0; i < state.eK; i++)
    {
        for (int j = 0; j < state.eK; j++)
        {
            //存在一个机器人能到达这两个站（这里的地图用的是chessBoard，chessBoard的墙面限制更加宽松，保障解集完备）
            if ((reachEnemyStation[0][i] && reachEnemyStation[0][j]) || (reachEnemyStation[1][i] && reachEnemyStation[1][j]) ||
                (reachEnemyStation[2][i] && reachEnemyStation[2][j]) || (reachEnemyStation[3][i] && reachEnemyStation[3][j]))
            {
                // 验证能否带货从一个站抵达另一个站，并计算距离（地图expandWall）
                if (a_star_search(state.enemyStation[i].p_ij[0] + state.enemyStation[i].p_ij[1] * 100, state.enemyStation[j].p_ij[0] + state.enemyStation[j].p_ij[1] * 100, 0, 0, 0, false))
                {
                    if ((state.enemyStation[i].type <= 7) && (state.enemyStation[i].type >= 4) &&
                        ((stationType[state.enemyStation[j].type].buyMaterials & (1 << state.enemyStation[i].type)) > 0))
                    {
                        s2sEnemyPair.push_back({ i,j });
                        s2sEnemyPath.push_back(path[0]);
                        s2sEnemyPriorty.push_back(-1); //预置
                        s2sEnemyRank.push_back(-1); //预置
                    }
                    s2sEnemyDist[i][j] = path[0].size() * 0.5;
                    maxEnemyDist = (maxEnemyDist < s2sEnemyDist[i][j]) ? s2sEnemyDist[i][j] : maxEnemyDist;
                }
                else
                {
                    s2sEnemyDist[i][j] = 10000; //没有路，给他个无穷大
                }
            }
            else
            {
                s2sEnemyDist[i][j] = 10000; //没有路，给他个无穷大
            }
        }
    }
    move_priorty_attacker();
}


/* 选择攻击者 */
void attacker_choose(int attackerAmount)
{
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        state.robot[rob_i].attacker = false;
    }

    //reachEnemyStation[4][50]
    int reachEnemyStationAmount[4] = { 0 };
    for (int i = 0; i < state.eK; i++)
    {
        reachEnemyStationAmount[0] += reachEnemyStation[0][i];
        reachEnemyStationAmount[1] += reachEnemyStation[1][i];
        reachEnemyStationAmount[2] += reachEnemyStation[2][i];
        reachEnemyStationAmount[3] += reachEnemyStation[3][i];
    }

    for (int i = 0; i < attackerAmount; i++)
    {
        int maxAmount = 0;
        int max_i = -1;
        for (int rob_i = 0; rob_i < 4; rob_i++)
        {
            if (maxAmount < reachEnemyStationAmount[rob_i])
            {
                maxAmount = reachEnemyStationAmount[rob_i];
                max_i = rob_i;
            }
        }
        if (max_i != -1)
        {
            reachEnemyStationAmount[max_i] = 0;
            state.robot[max_i].attacker = true;
            cmd[max_i].state = -1;
            cmd[max_i].takeStation = -1;
            cmd[max_i].sellStation = -1;
        }
        else
        {
            break;
        }
    }
}


//记录当前攻击点没有敌方来的持续帧数
int unattackTimer[4] = { 0 };
//记录当前攻击站点编号
int currentAttackEnemyStation[4] = { -1, -1, -1, -1 };

/* 攻击目标实时选择器，返回是否执行成功 */
void attack_target_choose(int rob_i) {

    //非攻击者直接返回false
    if (!state.robot[rob_i].attacker) {
        return;
    }

    //若无攻击目标，则分配一个
    if (currentAttackEnemyStation[rob_i] == -1) {
        for (int i = 0; i < keyEnemyStation.size(); i++) {
            if ((keyEnemyStation[i] != currentAttackEnemyStation[0])
                && (keyEnemyStation[i] != currentAttackEnemyStation[1])
                && (keyEnemyStation[i] != currentAttackEnemyStation[2])
                && (keyEnemyStation[i] != currentAttackEnemyStation[3])) {
                currentAttackEnemyStation[rob_i] = keyEnemyStation[i];
                attackerInPostion[rob_i] = false;
                unattackTimer[rob_i] = 0;
                break;
            }
        }
    }

    //判断是否就位
    if (attackerInPostion[rob_i] == false)
    {
        float p[2];
        p[0] = state.enemyStation[currentAttackEnemyStation[rob_i]].p[0];
        p[1] = state.enemyStation[currentAttackEnemyStation[rob_i]].p[1];
        if (TEAM == 0)
        {
            for (int i = 0; i < cornerEnemyStation.size(); i++)
            {
                if (cornerEnemyStation[i] == currentAttackEnemyStation[rob_i])
                {
                    switch (cornerEnemyStationKind[i])
                    {
                    case 0://右下是墙角
                        p[0] += -1;
                        p[1] += 1;
                        break;
                    case 1://左下是墙角
                        p[0] += 1;
                        p[1] += 1;
                        break;
                    case 2://左上是墙角
                        p[0] += 1;
                        p[1] += -1;
                        break;
                    case 3://右上是墙角
                        p[0] += -1;
                        p[1] += -1;
                        break;
                    }
                    break;
                }
            }
        }
        else
        {
            for (int i = 0; i < cornerEnemyStation.size(); i++)
            {
                if (cornerEnemyStation[i] == currentAttackEnemyStation[rob_i])
                {
                    switch (cornerEnemyStationKind[i])
                    {
                    case 0://右下是墙角
                        p[0] += 0.3;
                        p[1] += -0.3;
                        break;
                    case 1://左下是墙角
                        p[0] += -0.3;
                        p[1] += -0.3;
                        break;
                    case 2://左上是墙角
                        p[0] += -0.3;
                        p[1] += +0.3;
                        break;
                    case 3://右上是墙角
                        p[0] += +0.3;
                        p[1] += +0.3;
                        break;
                    }
                    break;
                }
            }
        }
        if (sqrt(pow(state.robot[rob_i].p[0] - p[0], 2) + pow(state.robot[rob_i].p[1] - p[1], 2)) < 0.54)
        {
            attackerInPostion[rob_i] = true;
            unattackTimer[rob_i] = 0;
        }
    }
    

    //就位后判断是否来敌
    if (attackerInPostion[rob_i] && (attackerState[rob_i] != 6)) {
        bool flag = false;
        
        //for (int i = 0; i < state.enemyRobot.size(); i++) {
        //    if (sqrt(pow(state.enemyRobot[i].p[0] - state.robot[rob_i].p[0],2)+pow(state.enemyRobot[i].p[1] - state.robot[rob_i].p[1],2)) < 0.45+0.53/2)            
        //    {
        //        //重置帧数
        //        flag = true;
        //        break;
        //    }
        //}
        for (int i = 0; i < 360; i++)
        {
            if ((state.robot[rob_i].lasersObject[i] == 1) && (state.robot[rob_i].lasers[i] < 0.45+0.1))
            {
                flag = true;
                break;
            }
        }
        if (flag) { unattackTimer[rob_i] = 0; }
        else { unattackTimer[rob_i]++; }

        //超时重选
        //if (unattackTimer[rob_i] > 50 * 15) {
        //    for (int i = 0; i < keyEnemyStation.size(); i++) {
        //        if ((keyEnemyStation[i] != currentAttackEnemyStation[0])
        //            && (keyEnemyStation[i] != currentAttackEnemyStation[1])
        //            && (keyEnemyStation[i] != currentAttackEnemyStation[2])
        //            && (keyEnemyStation[i] != currentAttackEnemyStation[3])) {
        //            currentAttackEnemyStation[rob_i] = keyEnemyStation[i];
        //            attackerInPostion[rob_i] = false;
        //            unattackTimer[rob_i] = 0;
        //            break;
        //        }
        //    }
        //}
    }

    outdebug("rob_[", rob_i, "] is ATTACKING station", currentAttackEnemyStation[rob_i]);
    return;
    //
}

//攻击者跟随换点
void attacker_follow(int rob_i)
{
    if (unattackTimer[rob_i] > 50 * 35)
    {
        for (int i = 0; i < keyEnemyStation.size(); i++) {
            if ((keyEnemyStation[i] != currentAttackEnemyStation[0])
                && (keyEnemyStation[i] != currentAttackEnemyStation[1])
                && (keyEnemyStation[i] != currentAttackEnemyStation[2])
                && (keyEnemyStation[i] != currentAttackEnemyStation[3])) {
                currentAttackEnemyStation[rob_i] = keyEnemyStation[i];
                attackerInPostion[rob_i] = false;
                unattackTimer[rob_i] = 0;
                attackerState[rob_i] = 0;
                break;
            }
        }
    }
    else if (unattackTimer[rob_i] > 50 * 30)
    {
        attackerState[rob_i] = 6;

        for (int emySta_i = 0; emySta_i < state.eK; emySta_i++)
        {
            if (sqrt(pow(state.robot[rob_i].p[0] - state.enemyStation[emySta_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyStation[emySta_i].p[1], 2)) < 3)
            {
                if (state.enemyStation[emySta_i].type <= 3) { continue; }
                bool tempFlag = true;
                for (int rob_k = 0; rob_k < 4; rob_k++)
                {
                    if (emySta_i == currentAttackEnemyStation[rob_k])
                    {
                        tempFlag = false;
                        break;
                    }
                }
                if (tempFlag)
                {
                    currentAttackEnemyStation[rob_i] = emySta_i;
                    unattackTimer[rob_i] = 0;
                    attackerInPostion[rob_i] = false;
                    attackerState[rob_i] = 0;
                    return;
                }
            }
        }

        //int start = x2col_chess(state.robot[rob_i].p[0]) + 101 * y2row_chess(state.robot[rob_i].p[1]);
        //int goal;
        //float pathSize = -1;
        //vector<int> followPath;
        //for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
        //{
        //    if (state.enemyRobot[emy_i].outTime < 30 * 50 - 10) { continue; }
        //    if (sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 5)
        //    { continue; }
        //    path[rob_i].clear();
        //    goal = x2col_chess(state.enemyRobot[emy_i].p[0]) + 101 * y2row_chess(state.enemyRobot[emy_i].p[1]);
        //    outdebug("In attacker_follow", state.enemyRobot[emy_i].p[0], state.enemyRobot[emy_i].p[1]);
        //    outdebug("In attacker_follow", start, goal);
        //    a_star_search(start, goal, 1, true, rob_i, false);
        //    if ((pathSize == -1) || (pathSize >= path[rob_i].size()))
        //    {
        //        pathSize = path[rob_i].size();
        //        followPath.clear();
        //        for (int i = 0; i < path[rob_i].size(); i++)
        //        {
        //            followPath.push_back(path[rob_i][i]);
        //        }
        //    }
        //    
        //}
        float dist = -1;
        int nearEnemy = -1;
        for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
        {
            if (state.enemyRobot[emy_i].outTime < 30 * 50 - 10) { continue; }
            //if (sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 5)
            //{ continue; }
            if ((nearEnemy == -1) || (dist >= sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2))))
            {
                dist = sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2));
                nearEnemy = emy_i;
            }
        }

        path[rob_i].clear();
        if (nearEnemy != -1)
        {
            int start = x2col_chess(state.robot[rob_i].p[0]) + 101 * y2row_chess(state.robot[rob_i].p[1]);
            int goal = x2col_chess(state.enemyRobot[nearEnemy].p[0]) + 101 * y2row_chess(state.enemyRobot[nearEnemy].p[1]);
            a_star_search(start, goal, 1, true, rob_i, false);
            path_simple(rob_i, 1, true, false);
        }
        else
        {
            unattackTimer[rob_i]++;
        }
    }
}

//攻击者追尾行动
void crazy_attacker_follow(int rob_i)
{
        attackerState[rob_i] = 6;

        float dist = -1;
        int nearEnemy = -1;
        for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
        {
            if ((nearEnemy == -1) || (dist >= sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2))))
            {
                if (30 * 50 - state.enemyRobot[emy_i].outTime > 50 * 3) { continue; } //不新鲜的敌方信息
                dist = sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2));
                nearEnemy = emy_i;
            }
        }

        outdebug("NearestEnemy", nearEnemy, state.enemyRobot[nearEnemy].p[0], state.enemyRobot[nearEnemy].p[0], state.enemyRobot[nearEnemy].outTime);

        path[rob_i].clear();
        if (nearEnemy != -1)
        {
            int start = x2col_chess(state.robot[rob_i].p[0]) + 101 * y2row_chess(state.robot[rob_i].p[1]);
            int goal = x2col_chess(state.enemyRobot[nearEnemy].p[0]) + 101 * y2row_chess(state.enemyRobot[nearEnemy].p[1]);
            a_star_search(start, goal, 1, true, rob_i, false);
            path_simple(rob_i, 1, true, false);
        }
        else
        {
            for (int i = 0; i < keyEnemyStation.size(); i++) {
                bool tempFlagNew = false;
                for (int rob_k = 0; rob_k < 4; rob_k++)
                {
                    if (rob_k == rob_i) { continue; }
                    if (keyEnemyStation[i] == currentAttackEnemyStation[rob_k])
                    {
                        tempFlagNew = true;
                        break;
                    }
                }
                if (tempFlagNew) { continue; }
                currentAttackEnemyStation[rob_i] = keyEnemyStation[i];
                int start = x2col_chess(state.robot[rob_i].p[0]) + 101 * y2row_chess(state.robot[rob_i].p[1]);
                int goal = x2col_chess(state.enemyStation[currentAttackEnemyStation[rob_i]].p[0]) + 101 * y2row_chess(state.enemyStation[currentAttackEnemyStation[rob_i]].p[1]);
                a_star_search(start, goal, 1, true, rob_i, false);
                path_simple(rob_i, 1, true, false);
                break;
            }
            /*currentAttackEnemyStation[rob_i] = keyEnemyStation[0];

            int start = x2col_chess(state.robot[rob_i].p[0]) + 101 * y2row_chess(state.robot[rob_i].p[1]);
            int goal = x2col_chess(state.enemyStation[currentAttackEnemyStation[rob_i]].p[0]) + 101 * y2row_chess(state.enemyStation[currentAttackEnemyStation[rob_i]].p[1]);
            a_star_search(start, goal, 1, true, rob_i, false);
            path_simple(rob_i, 1, true, false);*/
        }
}


void defensive_motion_control(int rob_i, float defensiveCoor[2])
{
    if ((old_defensiveCoor[rob_i][0] == defensiveCoor[0]) && (old_defensiveCoor[rob_i][1] == defensiveCoor[1]))
    {
        attackerState[rob_i] = old_attackerState[rob_i];
    }
    else
    {
        attackerState[rob_i] = 0;
    }

    //defensiveCoor[0] = state.enemyStation[enemyStationNum].p[0];
    //defensiveCoor[1] = state.enemyStation[enemyStationNum].p[1];

    outdebug("机器人号：", rob_i, "defensiveCoor=", defensiveCoor[0], defensiveCoor[1]);
    //己方机器人距站点的距离
    float robot2enmeyStation_dis = 0;
    //地方最近机器人距站点的距离
    float enemy2enmeyStation_dis = 16;
    //算的是距离的平方
    robot2enmeyStation_dis = (defensiveCoor[0] - state.robot[rob_i].p[0]) * (defensiveCoor[0] - state.robot[rob_i].p[0]) +
        (defensiveCoor[1] - state.robot[rob_i].p[1]) * (defensiveCoor[1] - state.robot[rob_i].p[1]);
    outdebug("机器人号：", rob_i, "robot2enmeyStation_dis=", robot2enmeyStation_dis);

    float p[2];
    p[0] = state.enemyStation[currentAttackEnemyStation[rob_i]].p[0];
    p[1] = state.enemyStation[currentAttackEnemyStation[rob_i]].p[1];

    float enemy_p[2] = { 0 };
    vector<float> robot2EnemyDistance;
    robot2EnemyDistance.resize(state.enemyRobot.size());
    if (state.enemyRobot.size() > 0)
    {
        for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
        {
            robot2EnemyDistance[emy_i] = sqrt((p[0] - state.enemyRobot[emy_i].p[0]) * (p[0] - state.enemyRobot[emy_i].p[0]) +
                (p[1] - state.enemyRobot[emy_i].p[1]) * (p[1] - state.enemyRobot[emy_i].p[1]));
            outdebug("enemy:", state.enemyRobot[emy_i].p[0], state.enemyRobot[emy_i].p[1]);
            outdebug("robot2EnemyDistance[", emy_i, "]=", robot2EnemyDistance[emy_i]);
        }
        // 找到最小值的迭代器
        auto it = min_element(robot2EnemyDistance.begin(), robot2EnemyDistance.end());
        // 计算索引
        int index = distance(robot2EnemyDistance.begin(), it);
        enemy2enmeyStation_dis = robot2EnemyDistance[index];
        if (enemy2enmeyStation_dis == 0)
        {
            enemy2enmeyStation_dis = 16;
        }
        if (enemy2enmeyStation_dis < 15)
        {
            enemy_p[0] = state.enemyRobot[index].p[0];
            enemy_p[1] = state.enemyRobot[index].p[1];
        }
        else
        {
            enemy_p[0] = 0;
            enemy_p[1] = 0;
        }
    }
    else
    {
        enemy2enmeyStation_dis = 16;
        enemy_p[0] = 0;
        enemy_p[1] = 0;
    }
    outdebug("机器人号：", rob_i, "enemy2enmeyStation_dis=", enemy2enmeyStation_dis);
    //状态机 0:从远方导航到堵截目标点0.54m范围内 1：走直线到达目标点0.05m = 0.05*0.05范围内 2：瞄向最近机器人，保持最大力的输出 3：倒退回到堵截目标点
    //状态的转换： 
    // 0->1  robot2enmeyStation_dis距离变小  1->2  最近机器人离目标点的距离小于1.5m  2->1/2 机器人远离了目标点 

    //if (robot2enmeyStation_dis > 1.1) { attackerState[rob_i] = 0; }

    if ((robot2enmeyStation_dis > 1.1) &&
        (((enemy_p[0] - state.robot[rob_i].p[0]) * (enemy_p[0] - state.robot[rob_i].p[0]) +
            (enemy_p[1] - state.robot[rob_i].p[1]) * (enemy_p[1] - state.robot[rob_i].p[1]) > 2))
        ) {
        attackerState[rob_i] = 0;
    }

    //if ( (robot2enmeyStation_dis < 0.9)&&(robot2enmeyStation_dis > 0.5)) { attackerState[rob_i] = 1; }

    switch (attackerState[rob_i])
    {
    case 0:
        /* code *///0.3是距离0.54的平方
        if (robot2enmeyStation_dis < 1.1) { attackerState[rob_i] = 1; }
        if ((robot2enmeyStation_dis < 1) && (enemy2enmeyStation_dis < 0.8))
        {
            attackerState[rob_i] = 1;
        }

        break;
    case 1:
        if ((robot2enmeyStation_dis < 0.025)) { 
            bool flag = false;
            for (int i = 0; i < cornerEnemyStation.size(); i++)
            {
                if (cornerEnemyStation[i] == currentAttackEnemyStation[rob_i])
                {
                    if (TEAM == 1)
                    {
                        flag = true;
                        break;
                    }
                }
            }
            if (flag) { break; }
            attackerState[rob_i] = 2;
        }
        // && (enemy2enmeyStation_dis < 15)
        break;
    case 2:
        if (enemy2enmeyStation_dis < 2.4) { attackerState[rob_i] = 3; }
        break;
    case 3:
        if ((robot2enmeyStation_dis > 0.16) && (enemy2enmeyStation_dis > 2.5)) { attackerState[rob_i] = 4; }
        //if ((robot2enmeyStation_dis > 0.16)&&(enemy2enmeyStation_dis<2.5)){ attackerState[rob_i] = 5; }
        break;
    case 4:
        // if (enemy2enmeyStation_dis<0.45) {attackerState[rob_i] = 3; }
        // break;
        if ((robot2enmeyStation_dis < 0.01) && (enemy2enmeyStation_dis >= 3)) { attackerState[rob_i] = 2; }
        if ((robot2enmeyStation_dis < 0.04) && (enemy2enmeyStation_dis < 3)) { attackerState[rob_i] = 3; }
        break;
    case 5:
        if (enemy2enmeyStation_dis < 0.45) { attackerState[rob_i] = 3; }
        break;
        if ((robot2enmeyStation_dis < 0.1) && (enemy2enmeyStation_dis < 2)) { attackerState[rob_i] = 3; }
        if (enemy2enmeyStation_dis > 2.5) { attackerState[rob_i] = 4; }
        //if((robot2enmeyStation_dis < 0.005)&&(enemy2enmeyStation_dis>2)&&(enemy2enmeyStation_dis<5) ){ attackerState[rob_i] = 2; }
        break;

    default:
        attackerState[rob_i] = 0;
        break;
    }

    outdebug("机器人号：", rob_i, "attackerState[rob_i]", attackerState[rob_i]);
    //状态0应当执行的操作
    if (attackerState[rob_i] == 0)//导航到堵截目标点附近
    {
        //
    }
    else if (attackerState[rob_i] == 1)
    {
        //状态1对准防守目标点，走直线过去
        rotate_control(rob_i, defensiveCoor);
        if (fabs(clamping_angle(rob_i, defensiveCoor)) > (PI / 64))
        {
            send_cmd_forward(rob_i, 0);
        }
        else
        {
            if (robot2enmeyStation_dis > 0.25)
            {
                send_cmd_forward(rob_i, 2);
            }
            else if ((robot2enmeyStation_dis < 0.25) && (robot2enmeyStation_dis > 0.15))
            {
                send_cmd_forward(rob_i, 1.1);
            }
            else if (robot2enmeyStation_dis < 0.15)
            {
                send_cmd_forward(rob_i, 0.5);
            }
            else if (robot2enmeyStation_dis < 0.05)
            {
                send_cmd_forward(rob_i, 0);
            }
        }
    }
    else if (attackerState[rob_i] == 2)
    {
        rotate_control(rob_i, enemy_p);
        send_cmd_forward(rob_i, 0);
    }
    else if (attackerState[rob_i] == 3)
    {
        rotate_control(rob_i, enemy_p);
        if ((robot2enmeyStation_dis > 0.9-(TEAM==1)*0.8) && (enemy2enmeyStation_dis > 1.1))
        {
            send_cmd_forward(rob_i, -0.01);//缓慢倒退
        }
        else
        {
            float rob_v = sqrt(state.robot[rob_i].speed[0] * state.robot[rob_i].speed[0] + state.robot[rob_i].speed[1] * state.robot[rob_i].speed[1]);
            rob_v = rob_v + 0.4;
            send_cmd_forward(rob_i, rob_v);
        }
    }
    else if (attackerState[rob_i] == 4)
    {
        float back_vector[2];
        back_vector[0] = -1 * (defensiveCoor[0] - state.robot[rob_i].p[0]);
        back_vector[1] = -1 * (defensiveCoor[1] - state.robot[rob_i].p[1]);
        float back_coor[2];
        back_coor[0] = state.robot[rob_i].p[0] + back_vector[0];
        back_coor[1] = state.robot[rob_i].p[1] + back_vector[1];
        if (fabs(clamping_angle(rob_i, back_coor)) < (PI / 64))
        {
            if (robot2enmeyStation_dis > 0.25)
            {
                send_cmd_forward(rob_i, -2);
            }
            else if ((robot2enmeyStation_dis < 0.25) && (robot2enmeyStation_dis > 0.15))
            {
                send_cmd_forward(rob_i, -1.1);
            }
            else if (robot2enmeyStation_dis < 0.15)
            {
                send_cmd_forward(rob_i, -0.5);
            }
            else if (robot2enmeyStation_dis < 0.05)
            {
                send_cmd_forward(rob_i, 0);
            }
        }
        else
        {
            send_cmd_forward(rob_i, -0.01);
        }
        rotate_control(rob_i, back_coor);
    }
    else if (attackerState[rob_i] == 5)
    {
        if (enemy2enmeyStation_dis < 2.0)
        {
            if (robot2enmeyStation_dis > 0.14)
            {
                send_cmd_forward(rob_i, -0.01);//缓慢倒退
            }
            else
            {
                float rob_v = sqrt(state.robot[rob_i].speed[0] * state.robot[rob_i].speed[0] + state.robot[rob_i].speed[1] * state.robot[rob_i].speed[1]);
                rob_v = rob_v + 0.4;
                send_cmd_forward(rob_i, rob_v);
            }
        }
        else
        {
            if (robot2enmeyStation_dis > 0.25)
            {
                send_cmd_forward(rob_i, -2);
            }
            else if ((robot2enmeyStation_dis < 0.25) && (robot2enmeyStation_dis > 0.15))
            {
                send_cmd_forward(rob_i, -1.1);
            }
            else if (robot2enmeyStation_dis < 0.15)
            {
                send_cmd_forward(rob_i, -0.05);
            }
            else if (robot2enmeyStation_dis < 0.05)
            {
                send_cmd_forward(rob_i, 0);
            }
        }

        float back_vector[2];
        back_vector[0] = -1 * (defensiveCoor[0] - state.robot[rob_i].p[0]);
        back_vector[1] = -1 * (defensiveCoor[1] - state.robot[rob_i].p[1]);
        float back_coor[2];
        back_coor[0] = state.robot[rob_i].p[0] + back_vector[0];
        back_coor[1] = state.robot[rob_i].p[1] + back_vector[1];
        rotate_control(rob_i, back_coor);
    }
    else
    {
        forward_rotate_control_APF(rob_i, defensiveCoor);
    }
    old_attackerState[rob_i] = attackerState[rob_i];
    old_defensiveCoor[rob_i][0] = defensiveCoor[0];
    old_defensiveCoor[rob_i][1] = defensiveCoor[1];
}

void attack_wheel()
{
    // 攻击者状态机
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if (state.robot[rob_i].attacker)
        {
            attack_target_choose(rob_i);
            float p[2];
            p[0] = state.enemyStation[currentAttackEnemyStation[rob_i]].p[0];
            p[1] = state.enemyStation[currentAttackEnemyStation[rob_i]].p[1];
            if (TEAM == 0)
            {
                for (int i = 0; i < cornerEnemyStation.size(); i++)
                {
                    if (cornerEnemyStation[i] == currentAttackEnemyStation[rob_i])
                    {
                        switch (cornerEnemyStationKind[i])
                        {
                        case 0://右下是墙角
                            p[0] += -1;
                            p[1] += 1;
                            break;
                        case 1://左下是墙角
                            p[0] += 1;
                            p[1] += 1;
                            break;
                        case 2://左上是墙角
                            p[0] += 1;
                            p[1] += -1;
                            break;
                        case 3://右上是墙角
                            p[0] += -1;
                            p[1] += -1;
                            break;
                        }
                        break;
                    }
                }
            }
            else
            {
                for (int i = 0; i < cornerEnemyStation.size(); i++)
                {
                    if (cornerEnemyStation[i] == currentAttackEnemyStation[rob_i])
                    {
                        switch (cornerEnemyStationKind[i])
                        {
                        case 0://右下是墙角
                            p[0] += 0.3;
                            p[1] += -0.3;
                            break;
                        case 1://左下是墙角
                            p[0] += -0.3;
                            p[1] += -0.3;
                            break;
                        case 2://左上是墙角
                            p[0] += -0.3;
                            p[1] += +0.3;
                            break;
                        case 3://右上是墙角
                            p[0] += +0.3;
                            p[1] += +0.3;
                            break;
                        }
                        break;
                    }
                }
            }
            if (attackerState[rob_i] != 6)
            {
                defensive_motion_control(rob_i, p);
            }
        }
    }
}
