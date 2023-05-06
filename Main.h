#pragma once
#include <vector>
using namespace std;

// 地图专用参数组
struct MapParams
{
    float L2; //二级货价值倍率
    float L3; //三级货价值倍率
    float SPEED; //真实最大速度系数
    float BETA; //距离惩罚影响因子
    float OMEGA; //原料紧缺程度影响因子
    float ETA; //掉头惩罚影响因子
    float ALPHA; //产品全局价值系数
    float MIU; //产品全局价值影响因子
};

// 工作站信息
struct StationType
{
    int type;   //类型&物品类型
    int needMaterials;  //生产配方0b00000000 7654321*
    int buyMaterials;   //收购材料0b00000000 7654321*
    int period; //工作周期
    int cost;   //购买价
    int sell;   //原始出售价
    int gain;   //该类型产品的差价（净利润）
    int quantity;  //该类工作台数量
    float productValue; //该类工作台产品的价值
};

// 工作站状态
struct StationState
{
    int type;   //类型
    float p[2]; //坐标
    int inBusy; //剩余生产时间 -1没有生产；0输出阻塞；+剩余时间
    int materials;  //原材料格状态0b0000000 7654321*
    bool finish;    //产品格状态
    int p_ij[2]; //方格位置
    int enemyOccupy; //是否被敌方占住？
    bool beenOccupy; //？
};

// 敌方工作站信息
struct StationEnemy
{
    int type;   //类型
    float p[2]; //坐标
    int p_ij[2]; //方格位置
};


// 机器人状态
struct RobotState
{
    int atStation;  //所处工作台id   -1没有工作台；0~K-1工作台下标
    int bring;  //携带物类型
    float timeValue;    //时间价值系数
    float collisionValue;   //碰撞价值系数
    float angularSpeed; //角速度
    float speed[2]; //线速度
    float direction;    //朝向
    float p[2]; //坐标
    int p_ij_init[2]; //初始方格位置
    float lasers[360]; //原始的激光雷达从机器人朝向逆时针顺序发射 360 条激光射线，取每条射线的最长照射长度
    float lasersMapDirection[360]; //原始的激光雷达对应下标的基于地图的角度
    float sumLaser; //雷达数值和
    float maxLaser_i; //最长激光所在下标
    float minLaser_i; //最短激光所在下标
    int lasersObject[360]; //每条激光所探测到的物体，0墙壁，1机器人
    bool attacker; //是否为攻击者
};

// 敌方机器人状态
struct EnemyState
{
    float p[2];
    int outTime;
};

// 帧状态
struct State
{
    int timeStamp;  //时间戳
    int cash;   //余额
    int K;  //工作站数量
    int eK; //敌方工作站数量
    StationState station[50];  //工作站状态
    StationEnemy enemyStation[50]; //敌方工作站信息
    RobotState robot[4];    //机器人状态
    vector<EnemyState> enemyRobot; //敌方机器人状态
};