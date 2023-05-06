#include <cmath>
#include "Motion.h"
#include "Global.h"
#include "Path.h"
#include "Attack.h"
#include "Out.h"
#include "Schedule.h"
#include "Calculate.h"
using namespace std;

Rotate_PID_param rotate_PID_param[4];

//掉头状态锁定变量
int rotate_model;
// 目标角度与当前航向夹角
float clampingAngle[4] = {0};


/* 角度PID控制初始化 */
void rotate_PID_init(int rob_i)
{
    rotate_PID_param[rob_i].err_r = 0;
    rotate_PID_param[rob_i].err_last_r = 0;
    rotate_PID_param[rob_i].Kp_r = 0.89;//具体值需要调试才能确定
    rotate_PID_param[rob_i].Ki_r = 0.0012;//
    rotate_PID_param[rob_i].Kd_r = 0.0;//
    rotate_PID_param[rob_i].integral_r = 0;
    rotate_PID_param[rob_i].v_r = 0;
    rotate_PID_param[rob_i].v_last_r = 0;
    rotate_PID_param[rob_i].old_target[0] = 0;
    rotate_PID_param[rob_i].old_target[1] = 0;
}

void motion_by_path()
{
    //运动控制
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        //攻击者
        if ((state.robot[rob_i].attacker) && (attackerState[rob_i] != 0) && (attackerState[rob_i] != 6))
        {
            continue;
        }
        // 无路径
        if (path[rob_i].size() == 0)
        {
            if (cmd[rob_i].takeStation != -1)
            {
                send_cmd_forward(rob_i, -2);
                send_cmd_rotate(rob_i, 0);
                continue;
            }
            else
            {
                //forward_rotate_control(rob_i, state.robot[rob_i].p);
                //forward_rotate_control_APF(rob_i, state.robot[rob_i].p);
                send_cmd_forward(rob_i, 0);
                send_cmd_rotate(rob_i, 0);
                continue;
            }
        }
        // 路径就一个点
        if (path[rob_i].size() == 1)
        {
            float p[2];
            if ((cmd[rob_i].state == 1) || (state.robot[rob_i].attacker))
            {
                p[0] = col2x_chess(path[rob_i][0] % 101);
                p[1] = row2y_chess(path[rob_i][0] / 101);
                p[0] += 0.25;
                p[1] -= 0.25;
            }
            else if (cmd[rob_i].state == 2)
            {
                p[0] = col2x(path[rob_i][0] % 100);
                p[1] = row2y(path[rob_i][0] / 100);
            }
            //forward_rotate_control(rob_i, p);
            forward_rotate_control_APF(rob_i, p);
            continue;
        }
        //路径有两个点
        if (path[rob_i].size() == 2)
        {
            float p[2];
            if ((cmd[rob_i].state == 1) || (state.robot[rob_i].attacker))
            {
                p[0] = col2x_chess(path[rob_i][1] % 101);
                p[1] = row2y_chess(path[rob_i][1] / 101);
                p[0] += 0.25;
                p[1] -= 0.25;
            }
            else if (cmd[rob_i].state == 2)
            {
                p[0] = col2x(path[rob_i][1] % 100);
                p[1] = row2y(path[rob_i][1] / 100);
            }
            if ((state.robot[rob_i].attacker) && (attackerState[rob_i] == 0))
            {
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
            }   
            outdebug("1620target:",path[rob_i][1],"坐标：",p[0],p[1]);
            //forward_rotate_control(rob_i, p);
            forward_rotate_control_APF(rob_i, p);
            //if (fabs(clamping_angle(rob_i, p)) < PI /8)
            //{
            //    send_cmd_forward(rob_i, 6 + (TEAM == 1));
            //}
            if (state.robot[rob_i].bring > 0)
            {
                if (state.station[cmd[rob_i].sellStation].enemyOccupy > 0)
                {
                    send_cmd_forward(rob_i, 6 + (TEAM == 1));
                }
            }
            else
            {
                if (state.station[cmd[rob_i].takeStation].enemyOccupy > 0)
                {
                    send_cmd_forward(rob_i, 6 + (TEAM == 1));
                }
            }
            continue;
        }
        //路径有一堆点
        float p[2];
        if ((cmd[rob_i].state == 1) || (state.robot[rob_i].attacker))
        {
            p[0] = col2x_chess(path[rob_i][1] % 101);
            p[1] = row2y_chess(path[rob_i][1] / 101);
        }
        else if (cmd[rob_i].state == 2)
        {
            p[0] = col2x(path[rob_i][1] % 100);
            p[1] = row2y(floor(path[rob_i][1] / 100));
        }
        //forward_rotate_control(rob_i, p);
        //forward_rotate_control_new(rob_i, p, path_state[rob_i]);
        forward_rotate_control_APF(rob_i, p);
        //if ((inSlow[rob_i] == true) && (nearestRobotDist[rob_i] < slowSpeedDist))//覆盖了前向控制命令，但是角度控制是原有的
        //{
        //    slowSpeed = 5.5;
        //    if (nearestRobotDist[rob_i] < 1.25)
        //    {
        //        slowSpeed = 3.5;
        //    }
        //    //slowSpeed = sqrt(2 * 14 * (slowSpeedDist));//nearestRobotDist[rob_i] + 0.1
        //    send_cmd_forward(rob_i, slowSpeed);
        //    //outdebug("rob_i, slowSpeed,nearestRobotDist:",rob_i, slowSpeed,nearestRobotDist[rob_i]);
        //}
    }

}


// APF
void forward_rotate_control_APF(int rob_i, float Target_Coord[2])//传入帧信息、机器人序号和目标点坐标
{
    //引入人工势场
    //首先计算周围8个探测点的坐标
    //外扩距离
    float Extend_distance;
    //最大加速度
    float Acceleration_forward, Acceleration_rotate, robot_mass, force_forward_max, force_rotate_max;
    float point_virtual_force_MAX;
    float robot_radius;

    // 力学参数
    if (state.robot[rob_i].bring == 0)
    {
        Extend_distance = 0.45 * 2.5; //半径外扩
        robot_radius = 0.45;
        Acceleration_forward = 19.6488;//每秒，直线加速度
        Acceleration_rotate = 38.8125;//角加速度
        if (TEAM == 0)
        {
            robot_mass = 12.7234; //质量
            point_virtual_force_MAX = 35; // 虚拟力最大值
            force_forward_max = 250; //前向力限制
            force_rotate_max = 50; //转向力限制
        }
        else if (TEAM == 1)
        {
            robot_mass = 9.5426;
            point_virtual_force_MAX = 25;
            force_forward_max = 187.5;
            force_rotate_max = 37.5;
        }

    }
    else
    {
        Extend_distance = 0.53 * 2.5;
        robot_radius = 0.53;
        Acceleration_forward = 14.1056;//每秒
        Acceleration_rotate = 20.1704;
        if (TEAM == 0)
        {
            robot_mass = 17.6494;
            point_virtual_force_MAX = 35;
            force_forward_max = 250;
            force_rotate_max = 50;
        }
        else if (TEAM == 1)
        {
            robot_mass = 13.2371;
            point_virtual_force_MAX = 25;
            force_forward_max = 187.5;
            force_rotate_max = 37.5;
        }
    }

    float forward_v_dt = 0; // 最后输出的加速度
    float rotate_v_dt = 0; // 最后输出的转向加速度

    float rotate_control_value, forward_control_value, robot_forward_force, robot_rotate_force;

    float robot_speed = sqrt(state.robot[rob_i].speed[0] * state.robot[rob_i].speed[0] + state.robot[rob_i].speed[1] * state.robot[rob_i].speed[1]);
    
    forward_control_value = forward_control(rob_i, Target_Coord); //原本的前向目标速度
    robot_forward_force = (forward_control_value - robot_speed) * 50 * robot_mass + 2; //原本的前向牵引力
    outdebug("机器人号：", rob_i, "forward_control_value:", forward_control_value, "robot_speed", robot_speed);
    // 修正force_forward_max
    if (robot_forward_force >= force_forward_max)
    {
        robot_forward_force = force_forward_max;
    }
    if (robot_forward_force <= (-1 * force_forward_max))
    {
        robot_forward_force = -1 * force_forward_max;
    }
    
    rotate_control_value = rotate_control(rob_i, Target_Coord);//原本的目标转速
    robot_rotate_force = (rotate_control_value - state.robot[rob_i].angularSpeed) * 50 * 0.5 * robot_mass * robot_radius * robot_radius;//原本的转动牵引力
    //修正force_rotate_max
    if (robot_rotate_force >= force_rotate_max)
    {
        robot_rotate_force = force_rotate_max;
    }
    if (robot_rotate_force <= (-1 * force_rotate_max))
    {
        robot_rotate_force = -1 * force_rotate_max;
    }

    // 墙壁和其他机器人引入的虚拟力
    //float virtual_force[30];
    float virtual_force_vector[30][2]; //各点（30个）
    float virtual_total_force_vector[2] = { 0,0 }; //合

    for (int i = 0; i < 30; i++)
    {
        int lasers_count = i * 12;
        if (state.robot[rob_i].lasers[lasers_count] < Extend_distance)
        {
            //在这里对虚拟力的大小进行给定 指数下降
            float virtual_force = (2 * state.robot[rob_i].lasersObject[lasers_count] + 0.1) * point_virtual_force_MAX * exp(-2 * state.robot[rob_i].lasers[lasers_count] / Extend_distance);
                //墙：0增益，机器人：1增益。exp(-2)，-2是衰减系数
            float angle = state.robot[rob_i].lasersMapDirection[lasers_count];
                //分力的角度+180度
            virtual_force_vector[i][0] = -1 * virtual_force * cos(angle);
            virtual_force_vector[i][1] = -1 * virtual_force * sin(angle);
        }
        else
        {
            virtual_force_vector[i][0] = 0;
            virtual_force_vector[i][1] = 0;
        }
        // 累积合力
        virtual_total_force_vector[0] = virtual_total_force_vector[0] + virtual_force_vector[i][0];
        virtual_total_force_vector[1] = virtual_total_force_vector[1] + virtual_force_vector[i][1];
    }
    //for循环完成得到合力
    //把力分解到机器人的朝向和垂直方向
    //计算虚拟合力与朝向的夹角
    outdebug("环境合力：", virtual_total_force_vector[0], virtual_total_force_vector[1]);

    // 虚拟
    float virtual_force_forward = virtual_total_force_vector[0] * cos(state.robot[rob_i].direction) + virtual_total_force_vector[1] * cos(PI / 2 - state.robot[rob_i].direction);
    float virtual_force_rotate = (-1) * virtual_total_force_vector[0] * sin(state.robot[rob_i].direction) + virtual_total_force_vector[1] * sin(PI / 2 - state.robot[rob_i].direction);
    outdebug("机器人号：", rob_i, "robot_forward_force:", robot_forward_force);
    outdebug("机器人号：", rob_i, "robot_rotate_force:", robot_rotate_force);
    outdebug("机器人号：", rob_i, "virtual_force_forward:", virtual_force_forward);
    outdebug("机器人号：", rob_i, "virtual_force_rotate:", virtual_force_rotate);

    // 虚拟 virtual_force_forward, virtual_force_rotate
    // 实际 robot_forward_force，robot_rotate_force
    // 求融合
    float forward_force_output = robot_forward_force + virtual_force_forward;
    float rotate_force_output = robot_rotate_force + virtual_force_rotate*0;
    //float forward_force_output = robot_forward_force;
    //float rotate_force_output = robot_rotate_force;
    forward_v_dt = (forward_force_output / robot_mass) / 50;
    rotate_v_dt = (rotate_force_output * 2) / (robot_mass * Extend_distance * Extend_distance) / 50;

    float forward_v_APF = robot_speed + forward_v_dt;
    float rotate_v_APF = state.robot[rob_i].angularSpeed + rotate_v_dt;

    outdebug("机器人号：", rob_i, "forward_v_APF", forward_v_APF, "rotate_v_APF", rotate_v_APF);
    if (fabs(forward_v_APF) < 0.04)
    {
        forward_v_APF = 0.1;
    }

    //输出速度指令
    send_cmd_forward(rob_i, forward_v_APF);
    //send_cmd_rotate(rob_i, rotate_v_APF);

}


//速度控制
float forward_control(int rob_i, float Target_Coord[2])
{
    float Map_Distance, Target_Distance, Map_Distance_D;
    float rob_v_max;
    float rob_v;
    float Acceleration_map;
    /*首先获取当前机器人坐标*/
    float Robot_Coord[2];
    /*然后通过计算获得目标向量*/
    float Target_Vector[2];
    float aa = 0.05;
    Robot_Coord[0] = state.robot[rob_i].p[0];
    Robot_Coord[1] = state.robot[rob_i].p[1];
    Target_Vector[0] = Target_Coord[0] - Robot_Coord[0];
    Target_Vector[1] = Target_Coord[1] - Robot_Coord[1];

    //计算当前速度
    //rob_v = sqrt(state.robot[rob_i].speed[0] * state.robot[rob_i].speed[0] + state.robot[rob_i].speed[1] * state.robot[rob_i].speed[1]);
    //计算距目标点距离
    Target_Distance = sqrt(Target_Vector[0] * Target_Vector[0] + Target_Vector[1] * Target_Vector[1]);

    if (state.robot[rob_i].bring == 0)//判断机器人有没有携带物品
    {
        Acceleration_map = 19.6488;//每秒
    }
    else
    {
        Acceleration_map = 14.1056;//每秒
    }

    float clampingAngle = clamping_angle(rob_i, Target_Coord);
    outdebug("机器人号：", rob_i, "前向控制clampingAngle=", clampingAngle);
    if (fabs(clampingAngle) < (PI * 1 / 8))
    {
        if (Target_Distance >= 1.0)
        {
            rob_v = 7;
        }
        if (Target_Distance <= 1.0)
        {
            rob_v = sqrt(2 * Acceleration_map * (Target_Distance));
        }
        if (Target_Distance < 0.2)
        {
            rob_v = 0.01;
        }
    }
    else
    {
        rob_v = 0.05;
    }
    outdebug("机器人号：", rob_i, "rob_v=", rob_v);

    send_cmd_forward(rob_i, rob_v);

    return rob_v;
}


//得到目标角度之后考虑进行转速控制
float rotate_control(int rob_i, float Target_Coord[2])
{
    float rotate_v_r;

    float clampingAngle = clamping_angle(rob_i, Target_Coord);
    outdebug("机器人号：", rob_i, "转向控制clampingAngle=", clampingAngle);
    //针对不同夹角的情况进行讨论,进行误差定义
    if (clampingAngle > (PI * 1.5 / 8))// && (rotate_model == 0)
    {
        //rotate_PID_param[rob_i].err_r = 5*tan(PI*3/8)*0.2;
        //直接给定转速
        rotate_PID_param[rob_i].v_r = -1 * PI;//-1 * 1 * 5 * tan(PI * 3 / 8) * 0.8
        rotate_PID_param[rob_i].Ki_r = 0.000;//
        rotate_PID_param[rob_i].integral_r = -0.0;//
        //rotate_model = 1;
    }
    else if (clampingAngle < (-1 * PI * 1.5 / 8))
    {
        //rotate_PID_param[rob_i].err_r = -5*tan(PI*3/8)*0.2;
        rotate_PID_param[rob_i].v_r = PI;//1 * 5 * tan(PI * 3 / 8) * 0.8
        rotate_PID_param[rob_i].Ki_r = 0.000;//
        rotate_PID_param[rob_i].integral_r = 0.0;//
        //rotate_model = 2;
    }
    else if (fabs(clampingAngle) <= (PI * 1.5 / 8))
    {
        //rotate_model = 0;
        rotate_PID_param[rob_i].err_r = 10 * tan(0 - clampingAngle);//不同情况下对err的正负要讨论 误差等于期望-测量值
        rotate_PID_param[rob_i].Ki_r = 0.001;//
        ////outdebug("rotate_PID_param[rob_i].err_r",rotate_PID_param[rob_i].err_r);
        //误差累积
        rotate_PID_param[rob_i].integral_r += rotate_PID_param[rob_i].err_r;

        //更换目标点，清零误差累积
        if ((rotate_PID_param[rob_i].old_target[0] != Target_Coord[0]) && (rotate_PID_param[rob_i].old_target[1] != Target_Coord[1]))
        {
            rotate_PID_param[rob_i].integral_r = 0;
            ////outdebug("integral_r = 0");
        }

        rotate_PID_param[rob_i].old_target[0] = Target_Coord[0];
        rotate_PID_param[rob_i].old_target[1] = Target_Coord[1];

        //PI算法实现
        rotate_PID_param[rob_i].v_r = rotate_PID_param[rob_i].Kp_r * rotate_PID_param[rob_i].err_r +
            rotate_PID_param[rob_i].Ki_r * rotate_PID_param[rob_i].integral_r +
            rotate_PID_param[rob_i].Kd_r * (rotate_PID_param[rob_i].err_r - rotate_PID_param[rob_i].err_last_r);
        rotate_PID_param[rob_i].v_r = 1 * rotate_PID_param[rob_i].v_r;
        ////outdebug("rotate_PID_param[rob_i].v_r", rotate_PID_param[rob_i].v_r);

        //误差传递
        rotate_PID_param[rob_i].err_last_r = rotate_PID_param[rob_i].err_r;
    }

    ////outdebug("clampingAngle[rob_i]", clampingAngle[rob_i]);


    rotate_v_r = rotate_PID_param[rob_i].v_r;
    //输出合法性检验
    if (rotate_v_r >= PI)
    {
        rotate_v_r = PI - 0.001;
    }
    if (rotate_v_r <= -1 * PI)
    {
        rotate_v_r = -1 * PI + 0.001;
    }
    //如果误差已经足够小了，让转速为0
    if (fabs(clampingAngle) < (PI / 900))
    {
        rotate_v_r = 0;
        outdebug("fabs(rotate_PID_param[rob_i].err_r) < 0.01");
    }
    //传递给前向速度控制
    rotate_PID_param[rob_i].v_r = rotate_v_r;

    //保存控制值
    rotate_PID_param[rob_i].v_last_r = rotate_v_r;

    //输出角速度指令
    send_cmd_rotate(rob_i, rotate_v_r);
    outdebug("机器人号：", rob_i, "rotate_v_r=", rotate_v_r);
    return rotate_v_r;
}

/* 运动控制（总）*/
void move()
{
    //运动控制
    for (int rob_i = 0; rob_i < 4; rob_i++)
    {
        // 守门员状态的攻击者
        if ((state.robot[rob_i].attacker) && (attackerState[rob_i] != 0) && (attackerState[rob_i] != 6))
        {
            continue;
        }
        // 无路径
        if (path[rob_i].size() == 0)
        {
            if (cmd[rob_i].takeStation != -1)
            {
                send_cmd_forward(rob_i, -2);
                send_cmd_rotate(rob_i, 0);
                continue;
            }
            else
            {
                //forward_rotate_control(rob_i, state.robot[rob_i].p);
                //forward_rotate_control_APF(rob_i, state.robot[rob_i].p);
                send_cmd_forward(rob_i, 0);
                send_cmd_rotate(rob_i, 0);
                continue;
            }
        }
        // 路径就一个点
        if (path[rob_i].size() == 1)
        {
            float p[2];
            if ((cmd[rob_i].state == 1) || (state.robot[rob_i].attacker))
            {
                p[0] = col2x_chess(path[rob_i][0] % 101);
                p[1] = row2y_chess(path[rob_i][0] / 101);
                p[0] += 0.25;
                p[1] -= 0.25;
            }
            else if (cmd[rob_i].state == 2)
            {
                p[0] = col2x(path[rob_i][0] % 100);
                p[1] = row2y(path[rob_i][0] / 100);
            }
            //forward_rotate_control(rob_i, p);
            forward_rotate_control_APF(rob_i, p);
            continue;
        }
        //路径有两个点
        if (path[rob_i].size() == 2)
        {
            float p[2];
            if ((cmd[rob_i].state == 1) || (state.robot[rob_i].attacker))
            {
                p[0] = col2x_chess(path[rob_i][1] % 101);
                p[1] = row2y_chess(path[rob_i][1] / 101);
                p[0] += 0.25;
                p[1] -= 0.25;
            }
            else if (cmd[rob_i].state == 2)
            {
                p[0] = col2x(path[rob_i][1] % 100);
                p[1] = row2y(path[rob_i][1] / 100);
            }
            if ((state.robot[rob_i].attacker) && (attackerState[rob_i] == 0))
            {
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
            }
            outdebug("target:", path[rob_i][1], "坐标：", p[0], p[1]);

            forward_rotate_control_APF(rob_i, p);
            //if (state.robot[rob_i].attacker == true)
            //{
            //    forward_control_cash(rob_i, p);
            //}

            // 路径上有敌人，直接冲
            //if (state.robot[rob_i].attacker == false)
            {
                float theta = clamping_angle(rob_i, p);
                theta = (theta < 0) ? (-theta) : (2 * PI - theta);
                theta = round(theta / PI * 180);
                theta = (theta == 360) ? 0 : theta;
                int k = theta;
                if ((state.robot[rob_i].lasersObject[k] == 1) && (state.robot[rob_i].lasers[k] < 5))
                {
                    for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
                    {
                        if (sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 1.5)
                        {
                            send_cmd_forward(rob_i, 6 + (TEAM == 1));
                        }
                    }
                }
            }

            // 目标站点上有敌人，加大马力直接冲
            if (state.robot[rob_i].attacker == false)
            {
                if (state.robot[rob_i].bring > 0)
                {
                    if (state.station[cmd[rob_i].sellStation].enemyOccupy > 0)
                    {
                        send_cmd_forward(rob_i, 6 + (TEAM == 1));
                    }
                }
                else
                {
                    if (state.station[cmd[rob_i].takeStation].enemyOccupy > 0)
                    {
                        send_cmd_forward(rob_i, 6 + (TEAM == 1));
                    }
                }
            }
            continue;
        }
        //路径有一堆点
        float p[2];
        if ((cmd[rob_i].state == 1) || (state.robot[rob_i].attacker))
        {
            p[0] = col2x_chess(path[rob_i][1] % 101);
            p[1] = row2y_chess(path[rob_i][1] / 101);
        }
        else if (cmd[rob_i].state == 2)
        {
            p[0] = col2x(path[rob_i][1] % 100);
            p[1] = row2y(floor(path[rob_i][1] / 100));
        }
        forward_rotate_control_APF(rob_i, p);

        // 路径上有敌人，直接冲
        if (state.robot[rob_i].attacker == false)
        {
            float theta = clamping_angle(rob_i, p);
            theta = (theta < 0) ? (-theta) : (2 * PI - theta);
            theta = round(theta / PI * 180);
            theta = (theta == 360) ? 0 : theta;
            int k = theta;
            if ((state.robot[rob_i].lasersObject[k] == 1) && (state.robot[rob_i].lasers[k] < 1))
            {
                for (int emy_i = 0; emy_i < state.enemyRobot.size(); emy_i++)
                {
                    if (sqrt(pow(state.robot[rob_i].p[0] - state.enemyRobot[emy_i].p[0], 2) + pow(state.robot[rob_i].p[1] - state.enemyRobot[emy_i].p[1], 2)) < 1.5)
                    {
                        send_cmd_forward(rob_i, 6 + (TEAM == 1));
                    }
                }
            }
        }
    }
}



float forward_control_cash(int rob_i, float Target_Coord[2])
{
    float Map_Distance, Target_Distance, Map_Distance_D;
    float rob_v_max;
    float rob_v;
    float Acceleration_map;
    /*首先获取当前机器人坐标*/
    float Robot_Coord[2];
    /*然后通过计算获得目标向量*/
    float Target_Vector[2];
    float aa = 0.05;
    Robot_Coord[0] = state.robot[rob_i].p[0];
    Robot_Coord[1] = state.robot[rob_i].p[1];
    Target_Vector[0] = Target_Coord[0] - Robot_Coord[0];
    Target_Vector[1] = Target_Coord[1] - Robot_Coord[1];

    //计算当前速度
    //rob_v = sqrt(state.robot[rob_i].speed[0] * state.robot[rob_i].speed[0] + state.robot[rob_i].speed[1] * state.robot[rob_i].speed[1]);
    //计算距目标点距离
    Target_Distance = sqrt(Target_Vector[0] * Target_Vector[0] + Target_Vector[1] * Target_Vector[1]);

    if (state.robot[rob_i].bring == 0)//判断机器人有没有携带物品
    {
        Acceleration_map = 19.6488;//每秒
    }
    else
    {
        Acceleration_map = 14.1056;//每秒
    }

    float clampingAngle = clamping_angle(rob_i, Target_Coord);
    outdebug("机器人号：", rob_i, "前向控制clampingAngle=", clampingAngle);

    if (fabs(clampingAngle) < (PI * 1 / 8))
    {
        // if (Target_Distance >= 1.0)
        // {
        //     rob_v = 7;
        // }
        // if (Target_Distance <= 1.0)
        // {
        //     rob_v = sqrt(2 * Acceleration_map * (Target_Distance));
        // }
        // if (Target_Distance < 0.2)
        // {
        //     rob_v = 0.01;
        // }
        rob_v = 7;
    }
    else
    {
        rob_v = 3.5;
    }
    outdebug("机器人号：", rob_i, "rob_v=", rob_v);

    send_cmd_forward(rob_i, rob_v);

    return rob_v;
}

