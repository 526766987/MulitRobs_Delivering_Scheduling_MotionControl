#include <cmath>
#include "Calculate.h"
#include "Global.h"
#include "Map.h"
#include "Schedule.h"

/* 根据输入的第几列算中心坐标 */
float col2x(int col) {
    return col * 0.5 + 0.25;
}

/* 根据输入的第几行算中心坐标 */
float row2y(int row) {
    return 50 - (row * 0.5 + 0.25);
}

/* 根据输入的坐标算第几列 */
int x2col(float x) {
    return floor(x / 0.5);
}

/* 根据输入的坐标算第几行 */
int y2row(float y) {
    return floor((50 - y) / 0.5);
}

/* 根据输入的第几列算中心坐标 */
int x2col_chess(float x)
{
    return floor((x + 0.25) / 0.5);
}

/* 根据输入的第几行算中心坐标 */
int y2row_chess(float y)
{
    return floor((50 - (y - 0.25)) / 0.5);
}

/* 根据输入的坐标算第几列 */
float col2x_chess(int col)
{
    return col * 0.5;
}

/* 根据输入的坐标算第几行 */
float row2y_chess(int row)
{
    return (100 - row) * 0.5;
}

/* 曼哈顿距离 */
float manhattan_distance(int p1_i, int p1_j, int p2_i, int p2_j)
{
    return (abs(p1_i - p2_i) + abs(p1_j - p2_j));
}

/* 计算机器人中心距离距离地图边缘的距离*/
float cal_map_distance_d(float robot_coor[2])
{
    float cal_Map_Distance_D;
    //定义机器人的坐标
    //float robot_coor[2];
    //定义与边框的四个距离
    float robot_Distance[4];

    robot_Distance[0] = fabs(0 - robot_coor[0]);
    robot_Distance[1] = fabs(50 - robot_coor[0]);
    robot_Distance[2] = fabs(0 - robot_coor[1]);
    robot_Distance[3] = fabs(50 - robot_coor[1]);

    //冒泡排序
    float temp = 0.0;
    int key = 0;
    while (key < 4)
    {
        for (int i = 3; i > key; i--)
        {
            //比较天平两端的大小，左大右则交换；否则不操作
            if (robot_Distance[i] < robot_Distance[i - 1])
            {
                temp = robot_Distance[i];
                robot_Distance[i] = robot_Distance[i - 1];
                robot_Distance[i - 1] = temp;
            }
        }
        //已排好序的元素不再参与排序
        key++;
    }

    cal_Map_Distance_D = robot_Distance[0];

    return cal_Map_Distance_D;
}

/* 沿当前朝向到地图边缘的距离 */
float cal_map_distance(int rob_i)
{
    float cla_Map_Distance, Map_cor_x = 0, Map_cor_y = 0;
    if (state.robot[rob_i].direction > (-1 * PI / 2) && state.robot[rob_i].direction < (PI / 2))
    {
        //计算x=50所代表的直线的交点
        Map_cor_y = tan(state.robot[rob_i].direction) * (50 - state.robot[rob_i].p[0]) + state.robot[rob_i].p[1];
        if (Map_cor_y >= 0 && Map_cor_y <= 50)
        {
            Map_cor_x = 50;
            Map_cor_y = Map_cor_y;
        }
        else if (Map_cor_y < 0)
        {
            Map_cor_x = state.robot[rob_i].p[0] + (0 - state.robot[rob_i].p[1]) / tan(state.robot[rob_i].direction);
            Map_cor_y = 0;
        }
        else if (Map_cor_y > 50)
        {
            Map_cor_x = state.robot[rob_i].p[0] + (50 - state.robot[rob_i].p[1]) / tan(state.robot[rob_i].direction);
            Map_cor_y = 50;
        }
    }
    else if ((state.robot[rob_i].direction > (-1 * PI / 2) && state.robot[rob_i].direction < (-1 * PI / 2)) || (state.robot[rob_i].direction > (PI / 2) && state.robot[rob_i].direction < PI))
    {

        //计算x=0所代表的直线的交点
        Map_cor_y = tan(state.robot[rob_i].direction) * (0 - state.robot[rob_i].p[0]) + state.robot[rob_i].p[1];
        if (Map_cor_y >= 0 && Map_cor_y <= 50)
        {
            Map_cor_x = 0;
            Map_cor_y = Map_cor_y;
        }
        else if (Map_cor_y < 0)
        {
            Map_cor_x = state.robot[rob_i].p[0] + (0 - state.robot[rob_i].p[1]) / tan(state.robot[rob_i].direction);
            Map_cor_y = 0;
        }
        else if (Map_cor_y > 50)
        {
            Map_cor_x = state.robot[rob_i].p[0] + (50 - state.robot[rob_i].p[1]) / tan(state.robot[rob_i].direction);
            Map_cor_y = 50;
        }
    }
    //计算以当前航向到地图边框之间的距离
    cla_Map_Distance = sqrt((Map_cor_x - state.robot[rob_i].p[0]) * (Map_cor_x - state.robot[rob_i].p[0]) + (Map_cor_y - state.robot[rob_i].p[1]) * (Map_cor_y - state.robot[rob_i].p[1]));

    return cla_Map_Distance;
}

/* 目标角度与当前航向夹角 */
float clamping_angle(int rob_i, float Target_Coord[2])
{
    /*首先获取当前机器人坐标*/
    float Robot_Coord[2];
    /*然后通过计算获得目标向量*/
    float Target_Vector[2];
    float rotate_v_r;
    float Alfa;
    float Beta = 0;

    Robot_Coord[0] = state.robot[rob_i].p[0];
    Robot_Coord[1] = state.robot[rob_i].p[1];

    Target_Vector[0] = Target_Coord[0] - Robot_Coord[0];
    Target_Vector[1] = Target_Coord[1] - Robot_Coord[1];

    /*然后计算当前航向与x轴正方向的夹角Alfa，目标航向与x轴正方向的夹角Beta*/
    Alfa = state.robot[rob_i].direction;

    //判断目标向量所处象限
    if (Target_Vector[0] > 0) //第一、四象限
    {
        Beta = atan(Target_Vector[1] / Target_Vector[0]);
    }
    else if ((Target_Vector[0] < 0) && (Target_Vector[1] >= 0))//第二象限 +
    {
        Beta = PI + atan(Target_Vector[1] / Target_Vector[0]);
    }
    else if ((Target_Vector[0] < 0) && (Target_Vector[1] < 0))//第三象限
    {
        Beta = -1 * PI + atan(Target_Vector[1] / Target_Vector[0]);
    }
    else if ((Target_Vector[0] == 0) && (Target_Vector[1] > 0))
    {
        Beta = PI / 2;
    }
    else if ((Target_Vector[0] == 0) && (Target_Vector[1] < 0))
    {
        Beta = -1 * PI / 2;
    }
    //于是二者相减得到夹角 或者说角度误差 期望-测量 期望为0 测量值为Alfa-Beta
    //clampingAngle[rob_i] = Beta - Alfa;
    float clampingAngle;
    if ((Beta - Alfa) >= PI)
    {
        clampingAngle = Alfa - Beta + 2 * PI;
    }
    else if ((Beta - Alfa) < (-1 * PI + 0.01))
    {
        clampingAngle = Alfa - Beta - 2 * PI;
    }
    else if (fabs(Beta - Alfa) < PI)
    {
        clampingAngle = Alfa - Beta;
    }
    else
    {
        clampingAngle = Alfa - Beta;
    }

    return clampingAngle; //得到的是角度测量值
}

/* 新版综合路程计算 */
float new_dist_cal(float distr2b, float distb2s, float theta) {
    return (pow((distr2b * (1 + (theta * mapparams[MAPKIND].ETA)) + distb2s), mapparams[MAPKIND].BETA));
}

/* 计算每种产品的全局价值：G × (1 - α^-k) × μ */
float cal_productValue(int type) {
    //该类型产品可卖到的工作台数量
    int quantity = 0;
    switch (type)
    {
    case 1:
        quantity = stationType[4].quantity + stationType[5].quantity;
        break;
    case 2:
        quantity = stationType[4].quantity + stationType[6].quantity;
        break;
    case 3:
        quantity = stationType[5].quantity + stationType[6].quantity;
        break;
    case 4:
        quantity = stationType[7].quantity + stationType[9].quantity;
        break;
    case 5:
        quantity = stationType[7].quantity + stationType[9].quantity;
        break;
    case 6:
        quantity = stationType[7].quantity + stationType[9].quantity;
        break;
    case 7:
        quantity = stationType[8].quantity + stationType[9].quantity;
        break;
    default:
        break;
    }
    //return (stationType[type].gain * (log10(quantity + 1) / log10(mapparams[map_num].ALPHA)) * mapparams[map_num].MIU);
    return (stationType[type].gain * (1 - pow(mapparams[MAPKIND].ALPHA, -quantity)) * mapparams[MAPKIND].MIU);
}

/* 用于计算{时间价值系数}和{碰撞价值系数}的函数f */
float f_func(float x, float maxX, float minRate)
{
    return (x < maxX) ? ((1.0f - sqrt(1.0f - pow(1.0f - x / maxX, 2))) * (1.0f - minRate) + minRate) : minRate;
}

/* 判断某工作台的原料格是否存在某类物品 */
bool pro_exsit(int type, int materials) {
    return materials & (1 << type);
}

/* 综合收入计算 */
float income_cal(int takeS, int sellS)
{
    // 直接现金流
    float income = stationType[state.station[takeS].type].gain;
    // 直接现金流的时间衰减
    income = income * s2sTimeLose[takeS][sellS];
    // 更高一级产品现金流
    float futureIncome = stationType[state.station[sellS].type].gain;
    // 判断一下，再加工是否急需该产品
    //判断卖出地是否是可再加工站
    if (state.station[sellS].type <= 7)
    {
        // 上面已有材料
        int haveRcv = 0;
        for (int i = 1; i < 8; i++)
        {
            haveRcv += (state.station[sellS].materials >> i) % 2;
        }
        // 已有材料越多，给的未来收益越大
        // 系数：0.1（无材料）,0.2（已收到1个）,0.3（已收到2个）
        futureIncome = futureIncome * (0.1 * (haveRcv + 1));
    }
    //如果不是可再加工地，预期收入为0
    else {
        futureIncome = 0;
    }
    //返回综合收入
    return (income + futureIncome);
}

/* 新版综合收入计算 */
float new_income_cal(int takeS, int sellS, float priorty)
{
    // 综合买入价格
    float buycost = stationType[state.station[takeS].type].cost * (1 - priorty);
    // 卖出地上面已有材料数
    int haveRcv = 0;
    //卖出地是否是可再加工站
    if (state.station[sellS].type <= 7)
    {
        for (int i = 1; i < 8; i++)
        {
            haveRcv += (state.station[sellS].materials >> i) % 2;
        }
    }
    // 综合出售价格
    float sellcost = stationType[state.station[takeS].type].sell * s2sTimeLose[takeS][sellS] * (1 + (haveRcv * mapparams[MAPKIND].OMEGA));
    // 返回综合利润
    return (sellcost - buycost);
}