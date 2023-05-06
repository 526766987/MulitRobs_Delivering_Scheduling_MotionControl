#include <vector>
#include <cmath>
#include "Schedule.h"
#include "Global.h"
#include "Path.h"
#include "Calculate.h"
#include "Out.h"
#include "Motion.h"
#include "Map.h"

using namespace std;

// 是否使用高级综合收益
bool newIncomeSwitch = false;

// 当前指令
schedulingCommond cmd[4] = {
    {-1,-1,1,0},
    {-1,-1,1,0},
    {-1,-1,1,0},
    {-1,-1,1,0}
};

// 上一次的历史指令
schedulingCommond cmdOld[4] = {
    {-1,-1,1,0},
    {-1,-1,1,0},
    {-1,-1,1,0},
    {-1,-1,1,0}
};

// 虚拟调度器输出指令
schedulingCommond cmdVis[4] = {
    {-1,-1,1,0},
    {-1,-1,1,0},
    {-1,-1,1,0},
    {-1,-1,1,0}
};

vector<vector<int>> s2sPath; //节点对中间路径
vector<vector<int>> s2sPair;  //节点对{i,j}
vector<float> s2sPriorty; //节点对重要度
vector<int> s2sRank; //操作等级
int actionRank[5] = { 5,4,3,2,1 }; //操作等级映射（7，456->7，123->456,456,123）
vector<vector<float>> s2sDist(50, vector<float>(50));  //节点距离K*K
vector<vector<float>> s2sTimeLose(50, vector<float>(50)); //理想时间衰减
float maxDist; //最远两个节点的距离max(s2sDist)

// 上一次的买卖站（仅在买卖发生时刷新，用作买卖失败后的重置保险） 0 take 1 sell
int lastOperationStation[4][2] = { {-1,-1},{-1,-1},{-1,-1},{-1,-1} };

//一直在执行相同的任务，说明有死锁可能，尝试让他横冲直撞
int sameCmdTime[4] = { 0,0,0,0 };

/* 站点捉对 */
void station_pair()
{
    // 地图泛洪
    all_flood(0);
    all_flood(1);

    maxDist = 0;
    for (int i = 0; i < state.K; i++)
    {
        for (int j = 0; j < state.K; j++)
        {
            //存在一个机器人能到达这两个站（这里的地图用的是chessBoard，chessBoard的墙面限制更加宽松，保障解集完备）
            if ((reachStation[1][0][i] && reachStation[1][0][j]) || (reachStation[1][1][i] && reachStation[1][1][j]) ||
                (reachStation[1][2][i] && reachStation[1][2][j]) || (reachStation[1][3][i] && reachStation[1][3][j]))
            {
                // 验证能否带货从一个站抵达另一个站，并计算距离（地图expandWall）
                if (a_star_search(state.station[i].p_ij[0] + state.station[i].p_ij[1] * 100, state.station[j].p_ij[0] + state.station[j].p_ij[1] * 100, 0, 0, 0,false))
                {
                    if ((state.station[i].type <= 7) &&
                        ((stationType[state.station[j].type].buyMaterials & (1 << state.station[i].type)) > 0))
                    {
                        s2sPair.push_back({ i,j });
                        s2sPath.push_back(path[0]);
                        s2sPriorty.push_back(-1); //预置
                        s2sRank.push_back(-1); //预置
                    }
                    s2sDist[i][j] = path[0].size() * 0.5;
                    s2sTimeLose[i][j] = f_func(s2sDist[i][j] / MAXV, 9000, 0.8); //群里有人问了，这里还是9000
                    maxDist = (maxDist < s2sDist[i][j]) ? s2sDist[i][j] : maxDist;
                }
                else
                {
                    s2sDist[i][j] = 10000; //没有路，给他个无穷大
                    s2sTimeLose[i][j] = 0;
                }
            }
            else
            {
                s2sDist[i][j] = 10000; //没有路，给他个无穷大
                s2sTimeLose[i][j] = 0;
            }
        }
    }
    move_priorty();
}

/* 站点对的权重计算 */
void move_priorty()
{
    int sta_i, sta_j;
    for (int pair_i = 0; pair_i < s2sPair.size(); pair_i++)
    {
        sta_i = s2sPair[pair_i][0];
        sta_j = s2sPair[pair_i][1];
        // 卖出7
        if (state.station[sta_i].type == 7)
        {
            s2sPriorty[pair_i] = 1 * (1 - s2sDist[sta_i][sta_j] / maxDist);
            s2sRank[pair_i] = actionRank[0];
        }
    }
    for (int pair_i = 0; pair_i < s2sPair.size(); pair_i++)
    {
        sta_i = s2sPair[pair_i][0];
        sta_j = s2sPair[pair_i][1];
        // 卖出456
        if ((state.station[sta_i].type <= 6) && (state.station[sta_i].type >= 4))
        {
            float upRank = -1;
            for (int pair_j = 0; pair_j < s2sPair.size(); pair_j++)
            {
                if (s2sPair[pair_j][0] == sta_j)
                {
                    upRank = (upRank < s2sPriorty[pair_j]) ? s2sPriorty[pair_j] : upRank;
                }
            }
            if (upRank == -1)
            {
                s2sPriorty[pair_i] = 1 * (1 - s2sDist[sta_i][sta_j] / maxDist);
                s2sRank[pair_i] = actionRank[3];
            }
            else
            {
                s2sPriorty[pair_i] = upRank * (1 - s2sDist[sta_i][sta_j] / maxDist);
                s2sRank[pair_i] = actionRank[1];
            }
        }
    }
    for (int pair_i = 0; pair_i < s2sPair.size(); pair_i++)
    {
        sta_i = s2sPair[pair_i][0];
        sta_j = s2sPair[pair_i][1];
        // 卖出123
        if ((state.station[sta_i].type <= 3))
        {
            float upRank = -1;
            for (int pair_j = 0; pair_j < s2sPair.size(); pair_j++)
            {
                if (s2sPair[pair_j][0] == sta_j)
                {
                    upRank = (upRank < s2sPriorty[pair_j]) ? s2sPriorty[pair_j] : upRank;
                }
            }
            if (upRank == -1)
            {
                s2sPriorty[pair_i] = 1 * (1 - s2sDist[sta_i][sta_j] / maxDist);
                s2sRank[pair_i] = actionRank[4];
            }
            else
            {
                s2sPriorty[pair_i] = upRank * (1 - s2sDist[sta_i][sta_j] / maxDist);
                s2sRank[pair_i] = actionRank[2];
            }
        }
    }
}

/* 调度器核心 */
void scheduler_core()
{
    //指令清空
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if ((cmd[rob_i].state == 1)) {
            cmd[rob_i].state = 0;
            cmd[rob_i].takeStation = -1;
            cmd[rob_i].sellStation = -1;
            cmd[rob_i].efficient = 0;
        }
    }

    //先把卖失败的机器人解决了
    bad_seller();

    /* 先做关键路径 */
    critical_taskpair();
    critical_scheduler();
    critical_taskpair();
    critical_scheduler();
    critical_taskpair();
    critical_scheduler();
    critical_taskpair();
    critical_scheduler();

    //若关键路径不够，则进行贪婪调度
    //关键路径的cmd.state会被临时设置为3，以防止被贪婪调度所覆盖，其余置1
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if ((cmd[rob_i].state == 0)) {
            cmd[rob_i].state = 1;
        }
    }

    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        // 在不考虑指令冲突的情况下进行虚拟调度
        if ((cmd[rob_i].state == 1)) {
            // 无货行驶
            real_time_greedy_scheduler(rob_i, newIncomeSwitch, true, -1);
        }
        else
        {
            // 带货行驶（指令不变）
            cmdVis[rob_i].takeStation = -1;
            cmdVis[rob_i].sellStation = -1;
            cmdVis[rob_i].efficient = -1;
        }
    }

    // 验证虚拟调度中是否有冲突
    bool flag = true;
    int round = 0;
    while (flag)
    {
        // 算法在5轮内依旧存在冲突，那么强制使用冲突调度
        if (round > 5) {
            for (int rob_i = 0; rob_i < 4; rob_i++)
            {
                if (cmd[rob_i].state == 1)
                {
                    real_time_greedy_scheduler(rob_i, newIncomeSwitch, false, -1);
                }
            }
            break;
        }

        flag = false;

        for (int rob_i = 0; rob_i < 4; rob_i++)
        {
            if (cmdVis[rob_i].sellStation == -1) { continue; }
            for (int rob_j = rob_i + 1; rob_j < 4; rob_j++)
            {
                if (cmdVis[rob_i].sellStation == -1) { continue; } //都是无货且有指令
                
                //rob_i与rob_j的冲突验证

                // 买入地冲突||卖出地+卖出项冲突
                if ((cmdVis[rob_i].takeStation == cmdVis[rob_j].takeStation) ||
                    ((cmdVis[rob_i].sellStation == cmdVis[rob_j].sellStation) && (state.station[cmdVis[rob_i].takeStation].type == state.station[cmdVis[rob_j].takeStation].type)))
                {
                    flag = true;
                    schedulingCommond cmdVis_i = cmdVis[rob_i];
                    schedulingCommond cmdVis_j = cmdVis[rob_j];

                    //plan A:替换掉rob_i的cmd
                    real_time_greedy_scheduler(rob_i, newIncomeSwitch, true, rob_j);
                    schedulingCommond cmdVis_i_new = cmdVis[rob_i];
                    cmdVis[rob_i] = cmdVis_i;

                    //plan B:替换掉rob_j的cmd
                    real_time_greedy_scheduler(rob_j, newIncomeSwitch, true, rob_i);
                    schedulingCommond cmdVis_j_new = cmdVis[rob_j];
                    cmdVis[rob_j] = cmdVis_j;

                    //比较plan
                    if ((cmdVis_i_new.efficient == 0) && (cmdVis_j_new.efficient == 0))
                    {
                        //这个命令是双方的唯一解，谁有优势谁做
                        if (cmdVis_j.efficient > cmdVis_i.efficient)
                        {
                            cmdVis[rob_i].takeStation = -1;
                            cmdVis[rob_i].sellStation = -1;
                            cmdVis[rob_i].efficient = 0;
                        }
                        else
                        {
                            cmdVis[rob_j].takeStation = -1;
                            cmdVis[rob_j].sellStation = -1;
                            cmdVis[rob_j].efficient = 0;
                        }
                    }
                    else if (cmdVis_i_new.efficient + cmdVis_j.efficient > cmdVis_j_new.efficient + cmdVis_i.efficient)
                    {
                        //把i换掉比把j换掉好
                        cmdVis[rob_i] = cmdVis_i_new;
                    }
                    else
                    {
                        //把j换掉比把i换掉好
                        cmdVis[rob_j] = cmdVis_j_new;
                    }
                }
            }
        }
        round++;
    }

    if (!flag)
    {
        for (int rob_i = 0; rob_i < 4; rob_i++)
        {
            if (cmd[rob_i].state == 1)
            {
                cmd[rob_i].sellStation = cmdVis[rob_i].sellStation;
                cmd[rob_i].takeStation = cmdVis[rob_i].takeStation;
                cmd[rob_i].efficient = cmdVis[rob_i].efficient;
            }
        }
    }

    //将关键路径的state还原回1
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if (cmd[rob_i].state == 3)
        {
            cmd[rob_i].state = 1;
        }
    }


}

/* 在工作台上进行状态轮转 */
void state_wheel()
{
    // 把闲置机器人的预跑动站点先清除，不能占别人的位置
    for (int rob_i = 0; rob_i < 4; rob_i++) {
        if (cmd[rob_i].sellStation == -1)
        {
            cmd[rob_i].takeStation = -1;
        }
        cmdVis[rob_i].takeStation = -1;
        cmdVis[rob_i].sellStation = -1;
        cmdVis[rob_i].efficient = 0;
    }

    // 带货状态，尝试卖出
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if ((cmd[rob_i].state == 2)) {
            //如果不同步（实际没带货物），则转换状态
            if (state.robot[rob_i].bring == 0) { cmd[rob_i].state = 1; continue; }
            //到卖出地才可以卖出
            if (state.robot[rob_i].atStation == cmd[rob_i].sellStation) {
                if (!pro_exsit(state.robot[rob_i].bring, state.station[cmd[rob_i].sellStation].materials)) {
                    // 卖出操作
                    send_cmd_sell(rob_i);

                    // 状态更新
                    cmd[rob_i].state = 1;
                    if (state.robot[rob_i].bring < 7) {
                        state.station[cmd[rob_i].sellStation].materials |= (1 << state.robot[rob_i].bring);
                    }
                    state.robot[rob_i].bring = 0;
                    rotate_PID_init(rob_i);

                    //防止没卖成功，记一下
                    lastOperationStation[rob_i][0] = cmd[rob_i].takeStation;
                    lastOperationStation[rob_i][1] = cmd[rob_i].sellStation;
                }
            }
        }
    }

    //开始新调度
    scheduler_core();

    //闲置机器人预跑动
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if ((cmd[rob_i].sellStation == -1) && (state.robot[rob_i].attacker == false))
        {
            if (real_time_runaway_scheduler(rob_i, true) == false)
            {
                real_time_runaway_scheduler(rob_i, false);
            }
        }
    }

    // 保存旧指令
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if ((cmdOld[rob_i].state == cmd[rob_i].state) &&
            (cmdOld[rob_i].takeStation == cmd[rob_i].takeStation) &&
            (cmdOld[rob_i].sellStation == cmd[rob_i].sellStation))
        {
            sameCmdTime[rob_i]++;
        }
        else
        {
            cmdOld[rob_i].state = cmd[rob_i].state;
            cmdOld[rob_i].takeStation = cmd[rob_i].takeStation;
            cmdOld[rob_i].sellStation = cmd[rob_i].sellStation;
            cmdOld[rob_i].efficient = cmd[rob_i].efficient;
            sameCmdTime[rob_i] = 0;
        }
    }

    // 无货状态，尝试买入
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        if ((cmd[rob_i].state == 1)) {
            // 如果发现身上还携带了东西的话，说明没有卖成功，还原历史指令
            if (state.robot[rob_i].bring != 0) {
                cmd[rob_i].state = 2;
                cmd[rob_i].takeStation = lastOperationStation[rob_i][0];
                cmd[rob_i].sellStation = lastOperationStation[rob_i][1];
                continue;
            }
            //若更新调度后的买入地就在此工作台，则直接买，不在则后面达到此条件
            if (state.robot[rob_i].atStation == cmd[rob_i].takeStation) {
                //强制判定剩余时间不可购买（最后保险）
                if (state.timeStamp > DEADLINE - 250) { continue; }
                //必须有东西才能买
                if (state.station[cmd[rob_i].takeStation].beenOccupy == true) { continue; }
                if (state.station[state.robot[rob_i].atStation].finish == 1) {
                    if (cmd[rob_i].sellStation == -1) { continue; } //为预跑动设置，只是预跑动，没有确定卖出点
                    if (state.station[cmd[rob_i].takeStation].beenOccupy) { continue; }
                    if (state.station[cmd[rob_i].sellStation].beenOccupy) { continue; }
                    //买入操作
                    send_cmd_buy(rob_i);

                    //状态更新
                    cmd[rob_i].state = 2;
                    state.station[cmd[rob_i].takeStation].finish = 0;
                    state.robot[rob_i].bring = state.station[cmd[rob_i].takeStation].type;
                    rotate_PID_init(rob_i);
                }
            }
        }
    }
}


/* 实时贪婪调度器 */
bool real_time_greedy_scheduler(int rob_i, bool newIncomeSwitch, bool visualSwitch, int rob_preempt)
{
    float efficient = 0;
    vector<int> bestPair = { -1,-1 };
    vector<vector<int>>::iterator iter;
    int iterIndex = 0;
    //对每种买卖点组合进行迭代
    for (iter = s2sPair.begin(); iter != s2sPair.end(); iter++) {
        if (state.station[(*iter)[0]].beenOccupy) { continue; }
        if (state.station[(*iter)[1]].beenOccupy) { continue; }
        //计算机器人目前到买入地的距离（直线距离判断）
        float r2tDist = sqrt(pow(state.robot[rob_i].p[0] - state.station[(*iter)[0]].p[0], 2) + pow(state.robot[rob_i].p[1] - state.station[(*iter)[0]].p[1], 2));
        //计算买卖地的距离（路线规划的城市距离）
        float t2sDist = s2sDist[(*iter)[0]][(*iter)[1]];

        //对买入地进行筛选
        //不能去没法到达的站点购买chessBoard
        if (reachStation[1][rob_i][(*iter)[0]] == false) { continue; }
        //如果目前没有产出，判断下面的情况
        if (state.station[(*iter)[0]].finish == 0) {
            //目前不生产的，跳过该组合
            if (state.station[(*iter)[0]].inBusy == -1) { continue; }
            //目前生产，但到达时也仍在生产的，跳过该组合
            if (state.station[(*iter)[0]].inBusy > r2tDist / MAXV) { continue; };
        }

        //对卖出地进行筛选
        //来不及卖的，跳过该组合
        if (((r2tDist + t2sDist) / (MAXV * mapparams[MAPKIND].SPEED)) * 50 > DEADLINE - state.timeStamp) { continue; }
        //卖出地如果原材料格已有买入材料，跳过该组合
        if ((state.station[(*iter)[1]].materials & (1 << state.station[(*iter)[0]].type)) > 0) { continue; }

        if (visualSwitch == false) // 真实调度
        {
            //机器人指令互斥
            bool flag = false;
            for (int i = 0; i < 4; i++)
            {
                //排除自己
                if (i == rob_i) { continue; }
                //不能和其他机器人到相同站取货
                if ((cmd[i].takeStation == (*iter)[0]) && (cmd[i].state == 1)) { flag = true; break; }
                //不能与其他机器人卖出相同货物到卖出地
                if ((cmd[i].sellStation == (*iter)[1]) && (state.station[cmd[i].takeStation].type == state.station[(*iter)[0]].type)) { flag = true; break; }
                // 关键路径取货
                if ((cmd[i].takeStation == (*iter)[0]) && (cmd[i].state == 3)) { flag = true; break; }
            }
            if (flag == true) { continue; }
        }
        else // 虚拟调度
        {
            if (rob_preempt >= 0)
            {
                //rob_preempt机器人的买入地排除
                if ((cmdVis[rob_preempt].takeStation == (*iter)[0]) && (cmdVis[rob_preempt].state == 1)) { continue; }
                //不能与rob_preempt机器人卖出相同货物到卖出地
                if ((cmdVis[rob_preempt].sellStation == (*iter)[1]) && (state.station[cmdVis[rob_preempt].takeStation].type == state.station[(*iter)[0]].type)) { continue; }
            }
            bool flag = false;
            for (int i = 0; i < 4; i++)
            {
                //排除自己
                if (i == rob_i) { continue; }
                //卖出地被运货态机器人预定，不可用
                if ((cmd[i].state == 2) && (cmd[i].sellStation == (*iter)[1]) && (state.station[cmd[i].takeStation].type == state.station[(*iter)[0]].type)) { flag = true; break; }
                // 关键路径取货
                if ((cmd[i].takeStation == (*iter)[0]) && (cmd[i].state == 3)) { flag = true; break; }
            }
            if (flag == true) { continue; }
        }

        //此组合为可行解，开始计算当前组合效益
        //当前组合真实收入+预期收入
        float income = income_cal((*iter)[0], (*iter)[1]);

        // 掉头惩罚
        float punishment = 0;
        float theta = atan2(state.station[(*iter)[0]].p[1] - state.robot[rob_i].p[1], state.station[(*iter)[0]].p[0] - state.robot[rob_i].p[0]);
        theta = abs(theta - state.robot[rob_i].direction);
        theta = (theta > PI) ? (2 * PI - theta) : theta;
        if (theta > PI * 2 / 3)
        {
            punishment = 5;
        }

        //当前组合效益
        float local_efficient;
        if (newIncomeSwitch)
        {
            local_efficient = (new_income_cal((*iter)[0], (*iter)[1], s2sPriorty[iterIndex]) + stationType[state.station[(*iter)[1]].type].productValue) / new_dist_cal(r2tDist, t2sDist, (fabs(clamping_angle(rob_i, state.station[(*iter)[0]].p)) / PI));
        }
        else
        {
            local_efficient = income / (r2tDist + t2sDist + punishment);
        }

        //当前组合效益比历史效益高，则更换为当前组合
        if (local_efficient > efficient)
        {
            efficient = local_efficient;
            bestPair[0] = (*iter)[0];
            bestPair[1] = (*iter)[1];
        }
        iterIndex++;
    }

    if (visualSwitch == false) // 真实调度
    {
        //将最佳组合赋值给cmd，若无解也会赋-1
        cmd[rob_i].takeStation = bestPair[0];
        cmd[rob_i].sellStation = bestPair[1];
        cmd[rob_i].efficient = efficient;
    }
    else
    {
        //将最佳组合赋值给cmdVis，若无解也会赋-1
        cmdVis[rob_i].takeStation = bestPair[0];
        cmdVis[rob_i].sellStation = bestPair[1];
        cmdVis[rob_i].efficient = efficient;
    }

    //如果无解，则返回false
    if (bestPair[0] == -1 || bestPair[1] == -1) {
        return false;
    }
    //最佳指令
    return true;
}


/* 实时闲置预跑动调度器 */
bool real_time_runaway_scheduler(int rob_i, bool chooseFinishStationSwitch)
{
    // 没有任务可做的机器人，让他先到最近的一个有产品且没人买的站候着，别管卖的地方了
    float efficient = 0;
    vector<int> bestPair = { -1,-1 };
    vector<vector<int>>::iterator iter;
    int iterIndex = 0;

    float minDist = 10000;
    //对每种买卖点组合进行迭代
    for (iter = s2sPair.begin(); iter != s2sPair.end(); iter++) {
        if (state.station[(*iter)[0]].beenOccupy) { continue; }
        if (state.station[(*iter)[1]].beenOccupy) { continue; }

        // 只去123
        if (state.station[(*iter)[0]].type > 3) { continue; }

        //计算机器人目前到买入地的距离（直线距离）
        float r2tDist = sqrt(pow(state.robot[rob_i].p[0] - state.station[(*iter)[0]].p[0], 2) + pow(state.robot[rob_i].p[1] - state.station[(*iter)[0]].p[1], 2));
        if (r2tDist >= minDist) { continue; }
        
        //对买入地进行筛选
        //不能去没法到达的站点购买（chessBoard）
        if (reachStation[1][rob_i][(*iter)[0]] == false) { continue; }
        if (chooseFinishStationSwitch)
        {
            //如果目前没有产出，判断下面的情况
            if (state.station[(*iter)[0]].finish == 0) {
                //目前不生产的，跳过该组合
                if (state.station[(*iter)[0]].inBusy == -1) { continue; }
                //目前生产，但到达时也仍在生产的，跳过该组合
                if (state.station[(*iter)[0]].inBusy > r2tDist / MAXV) { continue; };
            }
        }

        // 机器人指令冲突
        bool flag = false;
        for (int i = 0; i < 4; i++)
        {
            //排除自己
            if (i == rob_i) { continue; }
            //其他机器人的买入地排除
            if ((cmd[i].takeStation == (*iter)[0]) && (cmd[i].state == 1)) { flag = true; break; }
            //其他机器人的买入地排除
            if ((cmd[i].takeStation == (*iter)[0]) && (cmd[i].state == 3)) { flag = true; break; }
            // 不能把别人卖货的地方堵了
            if (cmd[i].sellStation == (*iter)[0]) { flag = true; break; }
        }
        if (flag == true) { continue; }

        // 记录下最佳点和距离
        bestPair[0] = (*iter)[0];
        minDist = r2tDist;

        iterIndex++;
    }

    //将最佳组合赋值给cmd，若无解也会赋-1
    cmd[rob_i].takeStation = bestPair[0];
    cmd[rob_i].sellStation = -1;
    cmd[rob_i].efficient = 0;

    //如果无解，则返回false
    if (bestPair[0] == -1) {
        return false;
    }
    //最佳指令
    return true;
}


vector<vector<int>> criticalPair; // 关键路径节点对
vector<float> criticalPairPriorty; //关键路径节点对重要度
vector<int> criticalPairRank; //关键路径节点对操作等级

/* 在确认sta_i站缺少mat_i材料后，寻找能供上的站点对*/
void find_criticalPair(int sta_i, int mat_i)
{
    //for (int rob_i = 0; rob_i < 4; rob_i++)
    //{
    //    // 虽然缺材料，但是已经有人承运了
    //    if ((cmd[rob_i].sellStation == sta_i) && (state.station[cmd[rob_i].takeStation].type == mat_i))
    //    {
    //        return;
    //    }
    //}
    //for (int i = 0; i < state.K; i++)
    //{
    //    // 虽然缺材料，但是已经在生产了
    //    if (state.station[i].type == mat_i)
    //    {
    //        if (state.station[i].inBusy > 0)
    //        {
    //            return;
    //        }
    //    }
    //}
    for (int pair_i = 0; pair_i < s2sPriorty.size(); pair_i++)
    {
        if (s2sPair[pair_i][1] != sta_i) { continue; }
        if (state.station[s2sPair[pair_i][0]].type != mat_i) { continue; }
        // 站点被占
        if (state.station[s2sPair[pair_i][0]].beenOccupy == true) { continue; }
        if (state.station[s2sPair[pair_i][1]].beenOccupy == true) { continue; }

        bool tempFlag = false;
        for (int cpair_i = 0; cpair_i < criticalPair.size(); cpair_i++)
        {
            // 运相同的东西到同一个站
            if ((s2sPair[pair_i][1] == criticalPair[cpair_i][1]) &&
                (state.station[s2sPair[pair_i][0]].type == state.station[criticalPair[cpair_i][0]].type))
            {
                if (s2sDist[s2sPair[pair_i][0]][s2sPair[pair_i][1]] < s2sDist[criticalPair[cpair_i][0]][criticalPair[cpair_i][1]])
                {
                    criticalPair.erase(criticalPair.begin() + cpair_i);
                    criticalPairPriorty.erase(criticalPairPriorty.begin() + cpair_i);
                    criticalPairRank.erase(criticalPairRank.begin() + cpair_i);
                    break;
                }
                else
                {
                    tempFlag = true;
                    break;
                }
            }
        }
        if (tempFlag == true) { continue; }

        criticalPair.push_back(s2sPair[pair_i]);
        criticalPairPriorty.push_back(s2sPriorty[pair_i]);
        criticalPairRank.push_back(s2sRank[pair_i]);

        if (state.station[sta_i].type == 7) {
            // 下级产品同样没产出，也缺原料
            if ((state.station[s2sPair[pair_i][0]].finish == 0) && (state.station[s2sPair[pair_i][0]].inBusy <= 0))
            {
                for (int next_mat_i = 1; next_mat_i <= 3; next_mat_i++)
                {
                    if (((state.station[s2sPair[pair_i][0]].materials & (1 << next_mat_i)) == 0) &&
                        ((stationType[state.station[s2sPair[pair_i][0]].type].needMaterials & (1 << next_mat_i)) > 0))
                    {
                        //下级产品缺原料
                        find_criticalPair(s2sPair[pair_i][0], next_mat_i);
                    }
                }
            }
        }
    }
}

/* 查询站7->8/9的priority */
float sell7_search(int sta_i)
{
    float highPriorty = 0;
    for (int pair_i = 0; pair_i < s2sPriorty.size(); pair_i++)
    {
        if (s2sPair[pair_i][0] == sta_i)
        {
            highPriorty = (highPriorty < s2sPriorty[pair_i]) ? s2sPriorty[pair_i] : highPriorty;
        }
    }
    return highPriorty;
}

/* 关键路径搜索 */
void critical_taskpair() {
    criticalPair.clear();
    criticalPairPriorty.clear();
    criticalPairRank.clear();
    vector<int> inHurry7; // 还没有开始生产的7
    vector<int> met7; // 已经拥有几个原料

    for (int sta_i = 0; sta_i < state.K; sta_i++)
    {
        if ((state.station[sta_i].type == 7) && (state.station[sta_i].finish == 1)) //把已经生产出的7优先卖掉
        {
            for (int pair_i = 0; pair_i < s2sPriorty.size(); pair_i++)
            {
                if (s2sPair[pair_i][0] == sta_i)
                {
                    // 站点被占
                    if (state.station[s2sPair[pair_i][0]].beenOccupy == true) { continue; }
                    if (state.station[s2sPair[pair_i][1]].beenOccupy == true) { continue; }
                    criticalPair.push_back(s2sPair[pair_i]);
                    criticalPairPriorty.push_back(s2sPriorty[pair_i]);
                    criticalPairRank.push_back(s2sRank[pair_i]);
                }
            }
        }

        //if ((state.station[sta_i].type == 7) && (state.station[sta_i].finish == 0) && (state.station[sta_i].inBusy <= 0)) //没有开始生产的7
        if (state.station[sta_i].type == 7)
        {
            // 站点被占
            if (state.station[sta_i].beenOccupy == true) { continue; }
            inHurry7.push_back(sta_i);
            int have = 0;
            for (int mat_i = 4; mat_i <= 6; mat_i++)
            {
                if ((state.station[sta_i].materials & (1 << mat_i)) > 0)
                {
                    have++;
                }
            }
            met7.push_back(sta_i);
        }
    }

    if (criticalPair.size() > 0)
    {
        for (int cpair_i = 0; cpair_i < criticalPair.size(); cpair_i++)
        {
            outdebug("criticalPair",criticalPair[cpair_i][0], criticalPair[cpair_i][1], criticalPairPriorty[cpair_i], criticalPairRank[cpair_i]);
        }
        return;
    }

    int maxMet = 0;
    int best7 = -1;
    float highPriorty = 0;
    for (int i = 0; i < inHurry7.size(); i++)
    {
        if ((maxMet < met7[i]) && (met7[i] != 3)) // 优先补齐拥有最多原材料的7
        {
            highPriorty = sell7_search(inHurry7[i]); // 7->8/9的优先级
            maxMet = met7[i];
            best7 = inHurry7[i];
        }
        else if ((maxMet == met7[i]) && (met7[i] != 3))
        {
            if (highPriorty < sell7_search(inHurry7[i])) // 7->8/9的优先级，高优先7优先补齐
            {
                highPriorty = sell7_search(inHurry7[i]);
                maxMet = met7[i];
                best7 = inHurry7[i];
            }
        }
    }

    if (best7 == -1) { return; }


    for (int mat_i = 4; mat_i <= 6; mat_i++)
    {
        if ((state.station[best7].materials & (1 << mat_i)) == 0)
        {
            find_criticalPair(best7, mat_i); // 寻找关键站点对
        }
    }
}

/* 测试旧指令是否存在冲突（能否还原旧指令） */
bool test_old_cmd(int rob_i)
{
    if (cmdOld[rob_i].state == 0) { return false; }
    if (cmdOld[rob_i].takeStation == -1) { return false; }
    if (cmdOld[rob_i].sellStation == -1) { return false; }
    if (state.station[cmd[rob_i].sellStation].beenOccupy) { return false; }

    int sta_i = cmdOld[rob_i].takeStation;
    int sta_j = cmdOld[rob_i].sellStation;

    bool flag = true;
    if (state.station[sta_i].finish == 0) {
        //目前生产，但到达时也仍在生产的，跳过该组合
        float dis = sqrt(pow(state.robot[rob_i].p[0] - state.station[sta_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.station[sta_i].p[1], 2));
        if ((state.station[sta_i].inBusy < (dis / MAXV)) && (state.station[sta_i].inBusy > 0))
        {
            //有空闲的机器人rob_i可以提前去接他
            //判断一下该空闲机器人是否来得及买卖
            //来不及卖的，跳过该组合
            if (((dis + s2sDist[sta_i][sta_j]) / (MAXV * mapparams[MAPKIND].SPEED)) * 50 > DEADLINE - state.timeStamp) { return false; }
            // 也就是说，该机器人空闲且有时间完成买卖
            flag = false;
        }
    }
    else
    {
        float dis = sqrt(pow(state.robot[rob_i].p[0] - state.station[sta_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.station[sta_i].p[1], 2));
        if (((dis + s2sDist[sta_i][sta_j]) / (MAXV * mapparams[MAPKIND].SPEED)) * 50 > DEADLINE - state.timeStamp) { return false; }
        // 也就是说，该机器人空闲且有时间完成买卖
        flag = false;
    }
    if (flag) { return false; } //生产不出来或者来不及

    if ((state.station[sta_i].materials & (1 << state.station[sta_i].type)) > 0) { return false; } //已有，无法收购

    flag = false;
    for (int i = 0; i < 4; i++)
    {
        if (i == rob_i) { continue; }
        if (((cmd[i].takeStation == sta_i) && (cmd[i].state == 1)) ||
            ((cmd[i].takeStation == sta_i) && (cmd[i].state == 3)) ||
            ((cmd[i].sellStation == sta_j) && (state.station[cmd[i].takeStation].type == state.station[sta_i].type)))
        {
            flag = true;
            break;
        }
    }
    if (flag) { return false; } //任务已被占有

    // 防止多个机器人一起冲向7/9
    if ((state.station[sta_j].type == 9) || (state.station[sta_j].type == 7))
    {
        int count = 0;
        for (int rob_i = 0; rob_i < 4; rob_i++)
        {
            if (cmd[rob_i].sellStation < 0) { continue; }
            if ((state.station[sta_j].type == state.station[cmd[rob_i].sellStation].type))
            {
                count++;
            }
        }
        if (cal_map_distance_d(state.station[sta_j].p) > 3)
        {
            if (count >= 3)
            {
                return false;
            }
        }
        else
        {
            if (count >= 1)
            {
                return false;
            }
        }
    }

    return true;
}

/* 关键路径调度器 */
void critical_scheduler()
{
    // 优先level
    vector<int> critical_rank_list = { 5,4,3,2,1 };
    //actionRank[0] = 5; // 7
    //actionRank[1] = 4; // 456->7
    //actionRank[2] = 3; // 123->456
    //actionRank[3] = 2; // 456
    //actionRank[4] = 1; // 123

    for (int rank_k = 0; rank_k < 5; rank_k++)
    {
        int rank = critical_rank_list[rank_k];
        float highest_priorty = 0;
        int sta_i = -1;
        int sta_j = -1;

        for (int pair_i = 0; pair_i < criticalPairRank.size(); pair_i++) {
            if (state.station[criticalPair[pair_i][0]].beenOccupy) { continue; }
            if (state.station[criticalPair[pair_i][1]].beenOccupy) { continue; }
            if (criticalPairRank[pair_i] == rank)
            {
                bool flag = true;
                if (state.station[criticalPair[pair_i][0]].finish == 0) {
                    //目前生产，但到达时也仍在生产的，跳过该组合
                    for (int rob_i = 0; rob_i < 4; rob_i++)
                    {
                        if (cmd[rob_i].state == 0)
                        {
                            //排除掉机器人到不了的位置
                            if (reachStation[1][rob_i][criticalPair[pair_i][0]] == false)
                            {
                                continue;
                            }
                            float dis = sqrt(pow(state.robot[rob_i].p[0] - state.station[criticalPair[pair_i][0]].p[0], 2) + pow(state.robot[rob_i].p[1] - state.station[criticalPair[pair_i][0]].p[1], 2));
                            if ((state.station[criticalPair[pair_i][0]].inBusy < (dis / MAXV)) && (state.station[criticalPair[pair_i][0]].inBusy > 0))
                            {
                                //有空闲的机器人rob_i可以提前去接他
                                //判断一下该空闲机器人是否来得及买卖
                                //来不及卖的，跳过该组合
                                if (((dis + s2sDist[criticalPair[pair_i][0]][criticalPair[pair_i][1]]) / (MAXV * mapparams[MAPKIND].SPEED)) * 50 > DEADLINE - state.timeStamp) { continue; }
                                // 也就是说，该机器人空闲且有时间完成买卖

                                //验证卖出地能否卖
                                if ((state.station[criticalPair[pair_i][1]].materials & (1 << state.station[criticalPair[pair_i][0]].type)) > 0) {
                                    //材料格阻塞，问问看有没有人能把成品买走，让格子空出来
                                    //到达时产品也没有被其他机器人买走，跳过该组合
                                    //搜索其他机器人有无将要在此购买产品的
                                    float willbuyTime;
                                    for (int i = 0; i < 4; i++)
                                    {
                                        //排除自己
                                        if (i == rob_i) { continue; }
                                        //若有，则计算其达到所需时间
                                        if (cmd[i].takeStation == criticalPair[pair_i][1]) {
                                            float distr2b2 = sqrt(pow(state.robot[i].p[0] - state.station[criticalPair[pair_i][1]].p[0], 2) + pow(state.robot[i].p[1] - state.station[criticalPair[pair_i][1]].p[1], 2));
                                            willbuyTime = distr2b2 / (MAXV * mapparams[MAPKIND].SPEED);
                                        }
                                        //若没有，则时间为很大
                                        willbuyTime = 10000;
                                    }
                                    if (((dis + s2sDist[criticalPair[pair_i][0]][criticalPair[pair_i][1]]) / (MAXV)) < willbuyTime) { continue; }
                                    //这样留下的是 到达时原料刚好空出来可以卖的组合
                                    flag = false;
                                }
                                else
                                {
                                    //材料格空缺，可以直接卖过去
                                    flag = false;
                                }
                            }
                        }
                    }
                }
                else
                {
                    //目前已产出
                    for (int rob_i = 0; rob_i < 4; rob_i++)
                    {
                        if (cmd[rob_i].state == 0)
                        {
                            //排除掉机器人到不了的位置
                            if (reachStation[1][rob_i][criticalPair[pair_i][0]] == false)
                            {
                                continue;
                            }
                            float dis = sqrt(pow(state.robot[rob_i].p[0] - state.station[criticalPair[pair_i][0]].p[0], 2) + pow(state.robot[rob_i].p[1] - state.station[criticalPair[pair_i][0]].p[1], 2));
                            //判断一下该空闲机器人是否来得及买卖
                            //来不及卖的，跳过该组合
                            if (((dis + s2sDist[criticalPair[pair_i][0]][criticalPair[pair_i][1]]) / (MAXV * mapparams[MAPKIND].SPEED)) * 50 > DEADLINE - state.timeStamp) { continue; }
                            // 也就是说，该机器人空闲且有时间完成买卖

                            //验证卖出地能否卖
                            if ((state.station[criticalPair[pair_i][1]].materials & (1 << state.station[criticalPair[pair_i][0]].type)) > 0) {
                                //材料格阻塞，问问看有没有人能把成品买走，让格子空出来
                                //到达时产品也没有被其他机器人买走，跳过该组合
                                //搜索其他机器人有无将要在此购买产品的
                                float willbuyTime;
                                for (int i = 0; i < 4; i++)
                                {
                                    //排除自己
                                    if (i == rob_i) { continue; }
                                    //若有，则计算其达到所需时间
                                    if (cmd[i].takeStation == criticalPair[pair_i][1]) {
                                        float distr2b2 = sqrt(pow(state.robot[i].p[0] - state.station[criticalPair[pair_i][1]].p[0], 2) + pow(state.robot[i].p[1] - state.station[criticalPair[pair_i][1]].p[1], 2));
                                        willbuyTime = distr2b2 / (MAXV * mapparams[MAPKIND].SPEED);
                                    }
                                    //若没有，则时间为很大
                                    willbuyTime = 10000;
                                }
                                if (((dis + s2sDist[criticalPair[pair_i][0]][criticalPair[pair_i][1]]) / (MAXV)) < willbuyTime) { continue; }
                                //这样留下的是 到达时原料刚好空出来可以卖的组合
                                flag = false;
                            }
                            else
                            {
                                //材料格空缺，可以直接卖过去
                                flag = false;
                            }
                        }
                    }
                }
                if (flag) { continue; } //生产不出来或者来不及

                //if ((state.station[criticalPair[pair_i][1]].materials & (1 << state.station[criticalPair[pair_i][0]].type)) > 0) { continue; } //已有，无法收购

                flag = false;
                for (int rob_i = 0; rob_i < 4; rob_i++)
                {
                    if (((cmd[rob_i].takeStation == criticalPair[pair_i][0]) && (cmd[rob_i].state == 1)) ||
                        ((cmd[rob_i].takeStation == criticalPair[pair_i][0]) && (cmd[rob_i].state == 3)) ||
                        ((cmd[rob_i].sellStation == criticalPair[pair_i][1]) && (state.station[cmd[rob_i].takeStation].type == state.station[criticalPair[pair_i][0]].type)))
                    {
                        flag = true;
                        break;
                    }
                }
                if (flag) { continue; } //任务已被占有

                // 防止多个机器人同时卖到7/9卡住
                if ((state.station[criticalPair[pair_i][1]].type == 9) || (state.station[criticalPair[pair_i][1]].type == 7))
                {
                    int count = 0;
                    for (int rob_i = 0; rob_i < 4; rob_i++)
                    {
                        if (cmd[rob_i].sellStation < 0) { continue; }
                        if ((state.station[criticalPair[pair_i][1]].type == state.station[cmd[rob_i].sellStation].type))
                        {
                            count++;
                        }
                    }
                    if (cal_map_distance_d(state.station[criticalPair[pair_i][1]].p) > 3)
                    {
                        if (count >= 3)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        if (count >= 1)
                        {
                            continue;
                        }
                    }
                }

                // 已有材料越多，越应优先做
                float advance = 0;
                if (state.station[criticalPair[pair_i][1]].type <= 7)
                {
                    // 上面已有材料
                    int haveRcv = 0;
                    for (int i = 1; i < 8; i++)
                    {
                        haveRcv += (state.station[criticalPair[pair_i][1]].materials >> i) % 2;
                    }
                    // 暂定为1.1（无材料）,1.2（已收到1个）,1.3（已收到2个）
                    advance += haveRcv + 1;
                }

                // 没有在生产的站点应优先填充材料
                if (state.station[criticalPair[pair_i][1]].type <= 7)
                {
                    if ((state.station[criticalPair[pair_i][1]].finish == 0) && (state.station[criticalPair[pair_i][1]].inBusy <= 0))
                    {
                        advance += 5;
                    }
                }

                // 综合优先度
                float priorty = criticalPairPriorty[pair_i] + advance;

                if (highest_priorty < priorty)
                {
                    highest_priorty = priorty;
                    sta_i = criticalPair[pair_i][0];
                    sta_j = criticalPair[pair_i][1];
                }
            }
        }

        if (sta_i != -1)
        {
            //寻找最合适的空闲机器人
            float min_d = 100;
            int  best_rob = -1;
            for (int rob_i = 0; rob_i < 4; rob_i++)
            {
                if (cmd[rob_i].state == 0)
                {
                    //排除掉机器人到不了的位置
                    if (reachStation[1][rob_i][sta_i] == false)
                    {
                        continue;
                    }

                    float d = sqrt(pow(state.robot[rob_i].p[0] - state.station[sta_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.station[sta_i].p[1], 2));

                    if (state.station[sta_i].finish == 1)
                    {
                        if (((d + s2sDist[sta_i][sta_j]) / (MAXV * mapparams[MAPKIND].SPEED)) * 50 > DEADLINE - state.timeStamp) { continue; }
                        if (d < min_d)
                        {
                            min_d = d;
                            best_rob = rob_i;
                        }
                    }
                    else
                    {
                        if (((d + s2sDist[sta_i][sta_j]) / (MAXV * mapparams[MAPKIND].SPEED)) * 50 > DEADLINE - state.timeStamp) { continue; }
                        if ((state.station[sta_i].inBusy < (d / MAXV)) && (state.station[sta_i].inBusy > 0))
                        {
                            if (d < min_d)
                            {
                                min_d = d;
                                best_rob = rob_i;
                            }
                        }
                    }
                }
            }

            // 找到了，判断下是否有价值切换到新的任务
            if (best_rob != -1)
            {
                cmd[best_rob].takeStation = sta_i;
                cmd[best_rob].sellStation = sta_j;
                cmd[best_rob].state = 3;

                int rob_i = best_rob;
                if (cmdOld[rob_i].state != 0)
                {
                    // 检查如果换任务的代价太大，距离上太亏就还原老决策
                    float dnew = sqrt(pow((state.station[cmd[rob_i].takeStation].p[0] - state.robot[rob_i].p[0]), 2) +
                        pow((state.station[cmd[rob_i].takeStation].p[0] - state.robot[rob_i].p[0]), 2));
                    float dold = sqrt(pow((state.station[cmdOld[rob_i].takeStation].p[0] - state.robot[rob_i].p[0]), 2) +
                        pow((state.station[cmdOld[rob_i].takeStation].p[0] - state.robot[rob_i].p[0]), 2));
                    if (dnew > dold * 3)
                    {
                        //判断旧指令是否还能做
                        if (test_old_cmd(rob_i))
                        {
                            //掉头代价太大，还原指令
                            cmd[rob_i].sellStation = cmdOld[rob_i].sellStation;
                            cmd[rob_i].takeStation = cmdOld[rob_i].takeStation;
                            cmd[rob_i].state = 1;
                            cmd[rob_i].efficient = cmdOld[rob_i].efficient;
                        }
                    }
                }
            }
            return;
        }
    }
    return;
}

//被堵卖出换点
int bad_seller_counting[4];
void bad_seller()
{
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        // 送货态，目标站被占领
        if ((state.robot[rob_i].bring > 0) && (state.station[cmd[rob_i].sellStation].beenOccupy == true))
        {
            int maySellStation = -1;
            float newDistance = 0;
            //遍历查询其它可售出站
            for (int pair_i = 0; pair_i < s2sPair.size(); pair_i++)
            {
                if (cmd[rob_i].takeStation != s2sPair[pair_i][0]) { continue; } // 必须是同一购买站
                if ((state.station[s2sPair[pair_i][1]].materials & (1 << state.robot[rob_i].bring)) != 0) { continue; } // 售出站的格子必须是空的
                if (state.station[s2sPair[pair_i][1]].beenOccupy == true) { continue; } //也没有被占领
                // 不是别人的卖出站
                bool tempFlag = false;
                for (int rob_k = 0; rob_k < 4; rob_k++)
                {
                    if (rob_i == rob_k) { continue; }
                    if ((cmd[rob_k].state == 2) && (cmd[rob_k].sellStation == s2sPair[pair_i][1]))
                    {
                        tempFlag = true;
                        break;
                    }
                }
                if (tempFlag) { continue; }

                if (maySellStation != -1)
                {
                    //就近卖掉
                    if (newDistance > s2sDist[cmd[rob_i].sellStation][s2sPair[pair_i][1]])
                    {
                        maySellStation = s2sPair[pair_i][1];
                        newDistance = s2sDist[cmd[rob_i].sellStation][s2sPair[pair_i][1]];
                    }
                }
                else
                {
                    maySellStation = s2sPair[pair_i][1];
                    newDistance = s2sDist[cmd[rob_i].sellStation][s2sPair[pair_i][1]];
                }
            }
            if (maySellStation != -1)
            {
                cmd[rob_i].sellStation = maySellStation;
                bad_seller_counting[rob_i] = 0;
            }
            else
            {
                bad_seller_counting[rob_i]++;
                if (bad_seller_counting[rob_i] > 800)
                {
                    send_cmd_destroy(rob_i);
                    bad_seller_counting[rob_i] = 0;
                }
            }
        }
        else
        {
            bad_seller_counting[rob_i] = 0;
        }
    }
}
