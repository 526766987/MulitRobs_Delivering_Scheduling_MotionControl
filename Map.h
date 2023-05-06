#pragma once

/*调参常量*/
static bool wallCorner[5] = { true,true,true,true,true }; //墙角是否扩展，针对expandWall图的调节

/*全局固定值声明*/
extern int MAPKIND; //地图类型 1,2,3,4
extern bool wallMap[100][100]; //墙地图
extern bool expandWall[100][100]; // 墙面扩展后的地图(带货态使用)
extern bool chessBoard[101][101]; // 围棋地图(取货态使用)

/*函数声明*/
void map_identify(); //地图识别
void wall_growth(bool cornerExpandSwitch); //构建expandWall地图
void chess_board_creat(); //构建chessBoard地图
bool cross(float p1[2], float p2[2], float p3[2], float p4[2]); //交叉判定（用于区分1和3）