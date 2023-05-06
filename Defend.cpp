#include <vector>
#include <cmath>
#include <numeric>
#include "Global.h"
#include "Calculate.h"
#include "Out.h"
#include "Attack.h"
#include "Map.h"


/* 敌方探测 */
vector<vector<float>> enemy_detect()
{
    float lasers_detection_point[8][360][2];
    float lasers_detection_distance = 100;
    //追击目标
    //float p_i_test[8] = { -1 };
    //float p_j_test[8] = { -1 };
    vector<vector<float>> p_enemy(8, vector<float>(2, -1));
    //float p_enemy[8][2] = { -1 };
    //int robot_point_count[4];
    vector<vector<int>> lasers_robot_point_index(4);//将检测到的机器人目标点索引记录下来
    lasers_robot_point_index.clear();
    vector<vector<float>> lasers_robot_point;

    lasers_robot_point.clear();

    // 分机器人处理雷达探测信息
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        float map_direction[360];
        int lasers_detection_grid[360][2];
        for (int j = 0; j < 360; j++)
        {
            if (state.robot[rob_i].lasers[j] <= lasers_detection_distance)
            {
                //先得到角度 
                map_direction[j] = state.robot[rob_i].direction + (static_cast<double>(j) / 360) * 2 * PI;
                //outdebug("map_direction[j]",j,"   ",map_direction[j]);
                if ((map_direction[j] >= -PI) && (map_direction[j] <= PI))
                {
                    map_direction[j] = map_direction[j];
                }
                else
                {
                    map_direction[j] = map_direction[j] - 2 * PI;
                }
                lasers_detection_point[rob_i][j][0] = state.robot[rob_i].p[0] + 1.001 * state.robot[rob_i].lasers[j] * cos(map_direction[j]);//乘以1.001是为了让点前伸到墙里面
                lasers_detection_point[rob_i][j][1] = state.robot[rob_i].p[1] + 1.001 * state.robot[rob_i].lasers[j] * sin(map_direction[j]);

                lasers_detection_grid[j][0] = x2col(lasers_detection_point[rob_i][j][0]);
                lasers_detection_grid[j][1] = y2row(lasers_detection_point[rob_i][j][1]);
                //得到的点既不在墙体内，也不是地图边缘，有机器人探测点
                if ((fabs(lasers_detection_point[rob_i][j][0] - 0) > 0.25) && (fabs(lasers_detection_point[rob_i][j][0] - 50) > 0.25) && (fabs(lasers_detection_point[rob_i][j][1] - 0) > 0.25) && (fabs(lasers_detection_point[rob_i][j][1] - 50) > 0.25))
                {
                    if (wallMap[lasers_detection_grid[j][0]][lasers_detection_grid[j][1]] == false)
                    {
                        //检测到一个机器人和检测到多个机器人
                        //robot_point_count[rob_i]++;
                        //outdebug("是不是墙",oldMap[lasers_detection_grid[j][0]][lasers_detection_grid[j][1]]);
                        //检测是不是在夹缝里
                        float Diagonal_point_test[4][2];
                        int Diagonal_grid_test[4][2];
                        bool Diagonal_point = true;
                        //向周围拓展四个点
                        Diagonal_point_test[0][0] = lasers_detection_point[rob_i][j][0] + 0.1;
                        Diagonal_point_test[0][1] = lasers_detection_point[rob_i][j][1] + 0.1;
                        Diagonal_point_test[1][0] = lasers_detection_point[rob_i][j][0] - 0.1;
                        Diagonal_point_test[1][1] = lasers_detection_point[rob_i][j][1] + 0.1;
                        Diagonal_point_test[2][0] = lasers_detection_point[rob_i][j][0] - 0.1;
                        Diagonal_point_test[2][1] = lasers_detection_point[rob_i][j][1] - 0.1;
                        Diagonal_point_test[3][0] = lasers_detection_point[rob_i][j][0] + 0.1;
                        Diagonal_point_test[3][1] = lasers_detection_point[rob_i][j][1] - 0.1;
                        for (int i = 0; i < 4; i++)//判断四个点是否在墙里
                        {
                            Diagonal_grid_test[i][0] = x2col(Diagonal_point_test[i][0]);
                            Diagonal_grid_test[i][1] = y2row(Diagonal_point_test[i][1]);
                            if ((Diagonal_grid_test[i][0] < 0) || (Diagonal_grid_test[i][1] < 0) || (Diagonal_grid_test[i][0] >= 100) || (Diagonal_grid_test[i][1] >= 100))
                            {
                                Diagonal_point = false;
                                break;
                            }
                            else
                            {
                                if (wallMap[Diagonal_grid_test[i][0]][Diagonal_grid_test[i][1]])//周围四个点有墙
                                {
                                    Diagonal_point = false;
                                    break;
                                }
                            }
                        }
                        if (Diagonal_point == true)
                        {
                            lasers_robot_point_index[rob_i].push_back(j);
                        }
                    }
                }
            }
            else
            {
                lasers_detection_point[rob_i][j][0] = -1;
                lasers_detection_point[rob_i][j][1] = -1;
            }

        }
        //outdebug(rob_i,"号机器人共检测到点：",lasers_robot_point_index[rob_i].size(),"个");
        // for (int i = 0; i < lasers_robot_point_index[rob_i].size(); i++)
        // {
        //     outdebug("lasers_robot_point_index[",rob_i,"][",i,"]=",lasers_robot_point_index[rob_i][i]);
        // }

    }
    //将四个机器人探测到的点进行合并
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        for (int i = 0; i < lasers_robot_point_index[rob_i].size(); i++)
        {
            float laser2self_distance[4];
            for (int j = 0; j < 4; j++)
            {
                laser2self_distance[j] = sqrt(pow(lasers_detection_point[rob_i][lasers_robot_point_index[rob_i][i]][0] - state.robot[j].p[0], 2) + pow(lasers_detection_point[rob_i][lasers_robot_point_index[rob_i][i]][1] - state.robot[j].p[1], 2));
            }
            //if ((laser2self_distance[0] > 1.06) && (laser2self_distance[1] > 1.06) && (laser2self_distance[2] > 1.06) && (laser2self_distance[3] > 1.06))//去掉激光打到己方机器人的点
            if ((laser2self_distance[0] > (0.45 + 0.08 * (state.robot[0].bring > 0))) && (laser2self_distance[1] > (0.45 + 0.08 * (state.robot[1].bring > 0))) && (laser2self_distance[2] > (0.45 + 0.08 * (state.robot[2].bring > 0))) && (laser2self_distance[3] > (0.45 + 0.08 * (state.robot[3].bring > 0))))
            {
                lasers_robot_point.push_back({ lasers_detection_point[rob_i][lasers_robot_point_index[rob_i][i]][0] , lasers_detection_point[rob_i][lasers_robot_point_index[rob_i][i]][1] });
            }
        }
    }
    // for (int i = 0; i < lasers_robot_point.size(); i++)
    // {
    //     outdebug("lasers_robot_point[",i,"]=",lasers_robot_point[i][0]," ",lasers_robot_point[i][1]);   
    // }

    int enemy_robots_num = 0;
    vector<bool> lasers_robot_point_state(lasers_robot_point.size());
    lasers_robot_point_state.assign(lasers_robot_point_state.size(), true);

    while (accumulate(lasers_robot_point_state.begin(), lasers_robot_point_state.end(), 0) > 0)
    {
        //计算所有点与第一个点的欧式距离 最远距离0.53*2
        for (int j = 0; j < lasers_robot_point.size(); j++)
        {
            if (lasers_robot_point_state[j] == true)
            {
                enemy_robots_num++;
                int num_in_cluster = 0;
                for (int i = 0; i < lasers_robot_point.size(); i++)
                {
                    float lasers_robot_point_dis = sqrt(pow(lasers_robot_point[i][0] - lasers_robot_point[j][0], 2) + pow(lasers_robot_point[i][1] - lasers_robot_point[j][1], 2));
                    if ((lasers_robot_point_dis <= (0.53 * 2)) && (lasers_robot_point_state[i] == true))
                    {
                        //p_i_test[enemy_robots_num - 1] = p_i_test[enemy_robots_num - 1] + lasers_robot_point[i][0];
                        //p_j_test[enemy_robots_num - 1] = p_j_test[enemy_robots_num - 1] + lasers_robot_point[i][1];
                        p_enemy[enemy_robots_num - 1][0] = p_enemy[enemy_robots_num - 1][0] + lasers_robot_point[i][0];
                        p_enemy[enemy_robots_num - 1][1] = p_enemy[enemy_robots_num - 1][1] + lasers_robot_point[i][1];
                        num_in_cluster++;
                        lasers_robot_point_state[i] = false;
                    }
                }

                if (num_in_cluster >= 2)
                {
                    p_enemy[enemy_robots_num - 1][0] = p_enemy[enemy_robots_num - 1][0] / num_in_cluster;
                    p_enemy[enemy_robots_num - 1][1] = p_enemy[enemy_robots_num - 1][1] / num_in_cluster;
                }
                else
                {
                    p_enemy[enemy_robots_num - 1][0] = 0;
                    p_enemy[enemy_robots_num - 1][1] = 0;
                    enemy_robots_num--;
                }
                //p_i_test[enemy_robots_num - 1] = p_i_test[enemy_robots_num - 1] / num_in_cluster;
                //p_j_test[enemy_robots_num - 1] = p_j_test[enemy_robots_num - 1] / num_in_cluster;
                //p_enemy[enemy_robots_num - 1][0] = p_enemy[enemy_robots_num - 1][0] / num_in_cluster;
                //p_enemy[enemy_robots_num - 1][1] = p_enemy[enemy_robots_num - 1][1] / num_in_cluster;
            }
        }
    }

    for (int j = enemy_robots_num; j < 8; j++)
    {
        //p_i_test[j] = -1;
        //p_j_test[j] = -1;
        p_enemy[j][0] = -1;
        p_enemy[j][1] = -1;
    }

    return p_enemy;
}

/* 搜索工作站角集 */
vector<int> cornerStation = {};
void corner_station()
{
    for (int sta_i = 0; sta_i < state.K; sta_i++)
    {
        int p_i = state.station[sta_i].p_ij[0];
        int p_j = state.station[sta_i].p_ij[1];

        if ((p_i + 2 < 100) && (p_j + 2 < 100) && (wallMap[p_i + 2][p_j + 0] + wallMap[p_i + 0][p_j + 2] == 2))
        {
            cornerStation.push_back(sta_i);
        }
        else if ((p_i - 2 >= 0) && (p_j + 2 < 100) && (wallMap[p_i - 2][p_j + 0] + wallMap[p_i + 0][p_j + 2] == 2))
        {
            cornerStation.push_back(sta_i);
        }
        else if ((p_i - 2 >= 0) && (p_j - 2 >= 0) && (wallMap[p_i - 2][p_j + 0] + wallMap[p_i + 0][p_j - 2] == 2))
        {
            cornerStation.push_back(sta_i);
        }
        else if ((p_i + 2 < 100) && (p_j - 2 >= 0) && (wallMap[p_i + 2][p_j + 0] + wallMap[p_i + 0][p_j - 2] == 2))
        {
            cornerStation.push_back(sta_i);
        }
    }
}

/* 雷达索敌与危险站确定 */
void scouting() {
    // 雷达搜索敌方位置（搜索、与上一帧的合并）
    vector<vector<float>> enemyPosition = enemy_detect();
    for (int i = 0; i < enemyPosition.size(); i++)
    {
        outdebug("enemy_detect():", enemyPosition[i][0], enemyPosition[i][1]);
    }
    for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
    {
        state.enemyRobot[emy_i].outTime--;
        if (state.enemyRobot[emy_i].outTime == 0)
        {
            state.enemyRobot.erase(state.enemyRobot.begin() + emy_i);
            emy_i--;
            continue;
        }
        for (int rob_i = 0; rob_i < 4; rob_i++)
        {
            float theta = clamping_angle(rob_i, state.enemyRobot[emy_i].p);
            theta = (theta < 0) ? (-theta) : (2 * PI - theta);
            theta = round(theta / PI * 180);
            theta = (theta == 360) ? 0 : theta;
            int k = theta;

            int r2sDist = sqrt(pow(state.enemyRobot[emy_i].p[1] - state.robot[rob_i].p[1], 2) + pow(state.enemyRobot[emy_i].p[0] - state.robot[rob_i].p[0], 2));
            if (state.robot[rob_i].lasers[k] >= r2sDist)
            {
                state.enemyRobot.erase(state.enemyRobot.begin() + emy_i);
                emy_i--;
                break;
            }
            if (r2sDist < 0.53 * 2)
            {
                state.enemyRobot.erase(state.enemyRobot.begin() + emy_i);
                emy_i--;
                break;
            }
        }
    }
    for (int new_emy_i = 0; new_emy_i < 8; new_emy_i++)
    {
        if (enemyPosition[new_emy_i][0] == -1)
        {
            continue;
        }
        if ((enemyPosition[new_emy_i][0] > 49.5) || (enemyPosition[new_emy_i][0] < 0.5) || (enemyPosition[new_emy_i][1] > 49.5) || (enemyPosition[new_emy_i][1] < 0.5))
        {
            continue;
        }
        bool oldEnemy = false;
        for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
        {
            //if (sqrt(pow(state.enemyRobot[emy_i].p[0] - enemyPosition[new_emy_i][0], 2) + pow(state.enemyRobot[emy_i].p[1] - enemyPosition[new_emy_i][1], 2)) < (6 + (TEAM == 1)) / 50 * (30 * 50 - state.enemyRobot[emy_i].outTime) + 1.06 * 2)
            if (sqrt(pow(state.enemyRobot[emy_i].p[0] - enemyPosition[new_emy_i][0], 2) + pow(state.enemyRobot[emy_i].p[1] - enemyPosition[new_emy_i][1], 2)) < 0.53)
            {
                state.enemyRobot[emy_i].p[0] = enemyPosition[new_emy_i][0] * (-1);
                state.enemyRobot[emy_i].p[1] = enemyPosition[new_emy_i][1] * (-1);
                state.enemyRobot[emy_i].outTime = 30 * 50;
                oldEnemy = true;
                break;
            }
        }
        if (oldEnemy == false)
        {
            EnemyState enemy;
            enemy.p[0] = enemyPosition[new_emy_i][0] * (-1);
            enemy.p[1] = enemyPosition[new_emy_i][1] * (-1);
            enemy.outTime = 30 * 50;
            state.enemyRobot.push_back(enemy);
        }
    }
    for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
    {
        if ((state.enemyRobot[emy_i].p[0] < 0) || (state.enemyRobot[emy_i].p[1] < 0))
        {
            state.enemyRobot[emy_i].p[0] *= -1;
            state.enemyRobot[emy_i].p[1] *= -1;
        }
    }
    outdebug("enemyAmount:", state.enemyRobot.size());

    // 搜索由敌方占领的工作台
    if (TEAM == 0) // 蓝队
    {
        //ban掉有至少两个敌方机器人占领的工作台（由于是多个机器人同时占领，因此判定半径需要大一点），同时解放能被友方机器人看到的工作台
        for (int sta_i = 0; sta_i < state.K; sta_i++)
        {
            if (state.station[sta_i].beenOccupy == true)
            {
                for (int rob_i = 0; rob_i < 4; rob_i++)
                {
                    float theta = clamping_angle(rob_i, state.station[sta_i].p);
                    theta = (theta < 0) ? (-theta) : (2 * PI - theta);
                    theta = round(theta / PI * 180);
                    theta = (theta == 360) ? 0 : theta;
                    int k = theta;

                    int r2sDist = sqrt(pow(state.station[sta_i].p[1] - state.robot[rob_i].p[1], 2) + pow(state.station[sta_i].p[0] - state.robot[rob_i].p[0], 2));
                    if (state.robot[rob_i].lasers[k] >= r2sDist + 1) { state.station[sta_i].beenOccupy = false; }
                }
            }
            state.station[sta_i].enemyOccupy = 0;
            for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
            {
                if (sqrt(pow(state.station[sta_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.station[sta_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 3)
                {
                    float d = sqrt(pow(state.station[sta_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.station[sta_i].p[1] - state.enemyRobot[emy_i].p[1], 2));
                    bool tempFlagNew = false;
                    for (int rob_k = 0; rob_k < 4; rob_k++)
                    {
                        float d1 = sqrt(pow(state.station[sta_i].p[0] - state.robot[rob_k].p[0], 2) + pow(state.station[sta_i].p[1] - state.robot[rob_k].p[1], 2));
                        if (d1 < d)
                        {
                            tempFlagNew = true;
                            break;
                        }
                    }
                    if (tempFlagNew) { continue; }
                    state.station[sta_i].enemyOccupy++;
                }
            }
            if (state.station[sta_i].enemyOccupy >= 2)
            {
                state.station[sta_i].beenOccupy = true;
            }
            // 自己的角站，要防住
            for (int i = 0; i < cornerStation.size(); i++)
            {
                state.station[sta_i].enemyOccupy = 0;
                for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
                {
                    if (sqrt(pow(state.station[sta_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.station[sta_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 1)
                    {
                        state.station[sta_i].enemyOccupy++;
                    }
                }
                if ((cornerStation[i] == sta_i) && (state.station[sta_i].enemyOccupy == 1))
                {
                    state.station[sta_i].beenOccupy = true;
                    break;
                }
            }
        }
    }
    else //红队
    {
        //ban掉有一个敌方机器人占领的工作台（一个占领，那么判定半径可以小），同时解放能被友方机器人看到的工作台
        for (int sta_i = 0; sta_i < state.K; sta_i++)
        {
            if (state.station[sta_i].beenOccupy == true)
            {
                for (int rob_i = 0; rob_i < 4; rob_i++)
                {
                    float theta = clamping_angle(rob_i, state.station[sta_i].p);
                    theta = (theta < 0) ? (-theta) : (2 * PI - theta);
                    theta = round(theta / PI * 180);
                    theta = (theta == 360) ? 0 : theta;
                    int k = theta;

                    int r2sDist = sqrt(pow(state.station[sta_i].p[1] - state.robot[rob_i].p[1], 2) + pow(state.station[sta_i].p[0] - state.robot[rob_i].p[0], 2));
                    if (state.robot[rob_i].lasers[k] >= r2sDist + 1) { state.station[sta_i].beenOccupy = false; }
                }
            }
            state.station[sta_i].enemyOccupy = 0;
            for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
            {
                if (sqrt(pow(state.station[sta_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.station[sta_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 1)
                {
                    float d = sqrt(pow(state.station[sta_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.station[sta_i].p[1] - state.enemyRobot[emy_i].p[1], 2));
                    bool tempFlagNew = false;
                    for (int rob_k = 0; rob_k < 4; rob_k++)
                    {
                        float d1 = sqrt(pow(state.station[sta_i].p[0] - state.robot[rob_k].p[0], 2) + pow(state.station[sta_i].p[1] - state.robot[rob_k].p[1], 2));
                        if (d1 < d)
                        {
                            tempFlagNew = true;
                            break;
                        }
                    }
                    if (tempFlagNew) { continue; }
                    state.station[sta_i].enemyOccupy++;
                }
            }
            if (state.station[sta_i].enemyOccupy >= 1)
            {
                state.station[sta_i].beenOccupy = true;
            }
            // 自己的角站，要防住
            for (int i = 0; i < cornerStation.size(); i++)
            {
                state.station[sta_i].enemyOccupy = 0;
                for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
                {
                    if (sqrt(pow(state.station[sta_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.station[sta_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 1)
                    {
                        state.station[sta_i].enemyOccupy++;
                    }
                }
                if ((cornerStation[i] == sta_i) && (state.station[sta_i].enemyOccupy == 1))
                {
                    state.station[sta_i].beenOccupy = true;
                    break;
                }
            }
        }
    }
}
