#include "Map.h"
#include "Global.h"
#include "Main.h"
#include "Path.h"
#include "Attack.h"
using namespace std;


int MAPKIND = 0; //地图类型初始定义为默认值
bool wallMap[100][100] = { false }; // 墙地图
bool expandWall[100][100] = { false }; // 墙面扩展后的地图
bool chessBoard[101][101] = { false }; // 围棋地图


/*识别是否为2类图*/
bool map2_identify()
{
	// Map 2
	//相对开阔的地形，各自的资源点都比较丰富，地图被分割为两个不连通区域，其中一个区域为红色基地，只有红色工作台，机器人初始为3红1蓝，另一个区域为蓝色基地，只有蓝色工作台，机器人初始为3蓝1红。
	//解读：这意味着双方各自有一个机器人完全不需要考虑运输物品，只需要作为对方的移动障碍。
	
	//注：用工作台连通性去识别是最稳妥的
	
	bool attacker = false;
	int reachOurStationAmount[4] = { 0 };
	int reachEnemyStationAmount[4] = { 0 };
	for (int rob_i = 0; rob_i < 4; rob_i++)
	{
		for (int sta_i = 0; sta_i < state.K; sta_i++)
		{
			reachOurStationAmount[rob_i] += reachStation[1][rob_i][sta_i];
		}
		for (int sta_i = 0; sta_i < state.eK; sta_i++)
		{
			reachEnemyStationAmount[rob_i] += reachEnemyStation[rob_i][sta_i];
		}

		if ((reachOurStationAmount[rob_i] == state.K) && (reachEnemyStationAmount[rob_i] == 0))
		{
			// 工作者
		}
		else if ((reachOurStationAmount[rob_i] == 0) && (reachEnemyStationAmount[rob_i] == state.eK))
		{
			// attacker
			if (attacker == false) { attacker = true; } //找到了attacker
			else { return false; } //map2不可能会出现第二个attacker
		}
		else
		{
			//绝对不是图2
			return false;
		}
	}
	return true;
}

/*识别是否为4类图*/
bool map4_identify()
{
	// Map 4
	//极为开阔的地形，整张地图完全没有蓝色工作台，只有红色工作台，且红色工作台点比较丰富。
	//解读： 这意味着蓝方完全不需要考虑运输，只需要作为红方的移动障碍。

	//注：用工作台数量识别

	if ((state.K == 0) || (state.eK == 0))
	{
		return true;
	}
	return false;
}

/*识别是否是图1，否则为图3*/
bool cross(float p1[2], float p2[2], float p3[2], float p4[2])
{
    bool flag = true;
    if ((p1[0] > p2[0] ? p1[0] : p2[0]) < (p3[0] < p4[0] ? p3[0] : p4[0]) ||
        (p1[1] > p2[1] ? p1[1] : p2[1]) < (p3[1] < p4[1] ? p3[1] : p4[1]) ||
        (p3[0] > p4[0] ? p3[0] : p4[0]) < (p1[0] < p2[0] ? p1[0] : p2[0]) ||
        (p3[1] > p4[1] ? p3[1] : p4[1]) < (p1[1] < p2[1] ? p1[1] : p2[1]))
    {
        flag = false;
    }
    if ((((p1[0] - p3[0]) * (p4[1] - p3[1]) - (p1[1] - p3[1]) * (p4[0] - p3[0])) *
        ((p2[0] - p3[0]) * (p4[1] - p3[1]) - (p2[1] - p3[1]) * (p4[0] - p3[0]))) > 0 ||
        (((p3[0] - p1[0]) * (p2[1] - p1[1]) - (p3[1] - p1[1]) * (p2[0] - p1[0])) *
            ((p4[0] - p1[0]) * (p2[1] - p1[1]) - (p4[1] - p1[1]) * (p2[0] - p1[0]))) > 0)
    {
        flag = false;
    }

    return flag;
}

bool map1_not3_identify()
{
	//Map 1
	//相对开阔的地形，各自的资源点都比较丰富，双方运输路径存在交错。（参考练习赛图1，但地形更开阔，资源点更丰富）
	//解读：完全无避让的话会存在相互冲撞，这意味着双方在行进过程中需要根据持物状态来决定是否避让。

	//Map 3
	//相对狭窄的地形，双方有各自独立基地，基地之间存在多条路径连通，并且基地外部存在一些分散的红蓝工作台可使用。（参考练习赛图2）
	//解读：双方可选择各自发展，或者干扰对方移动，或者占领一些关键点位。

	//分析：
	//对于地形开阔/狭窄的判断是不可靠的
	//考虑使用运输路径交错来判断

	//做法：
	//对123求位置平均点p1，456求位置平均点p2
	//我方p1->p2与敌方p1->p2存在交叉，则为map 1

	float p123Our[2], p456Our[2], p7Our[2], p123Enemy[2], p456Enemy[2], p7Enemy[2] = {0};
	int count123Our, count456Our, count7Our, count123Enemy, count456Enemy, count7Enemy = 0;
	for (int sta_i = 0; sta_i < state.K; sta_i++)
	{
		if (state.station[sta_i].type <= 3)
		{
			p123Our[0] += state.station[sta_i].p[0];
			p123Our[1] += state.station[sta_i].p[1];
			count123Our++;
		}
		else if (state.station[sta_i].type <= 6)
		{
			p456Our[0] += state.station[sta_i].p[0];
			p456Our[1] += state.station[sta_i].p[1];
			count456Our++;
		}
        else if (state.station[sta_i].type == 7)
        {
            p7Our[0] += state.station[sta_i].p[0];
            p7Our[1] += state.station[sta_i].p[1];
            count7Our++;
        }
	}

	p123Our[0] /= count123Our;
	p123Our[1] /= count123Our;
	p456Our[0] /= count456Our;
	p456Our[1] /= count456Our;
    if (count7Our > 0)
    {
        p7Our[0] /= count7Our;
        p7Our[1] /= count7Our;
    }

	for (int sta_i = 0; sta_i < state.eK; sta_i++)
	{
		if (state.enemyStation[sta_i].type <= 3)
		{
			p123Enemy[0] += state.enemyStation[sta_i].p[0];
			p123Enemy[1] += state.enemyStation[sta_i].p[1];
			count123Enemy++;
		}
		else if (state.enemyStation[sta_i].type <= 6)
		{
			p456Enemy[0] += state.enemyStation[sta_i].p[0];
			p456Enemy[1] += state.enemyStation[sta_i].p[1];
			count456Enemy++;
		}
        else if (state.enemyStation[sta_i].type == 7)
        {
            p7Enemy[0] += state.enemyStation[sta_i].p[0];
            p7Enemy[1] += state.enemyStation[sta_i].p[1];
            count7Enemy++;
        }
	}

	p123Enemy[0] /= count123Enemy;
	p123Enemy[1] /= count123Enemy;
    p456Enemy[0] /= count456Enemy;
    p456Enemy[1] /= count456Enemy;
    if (count7Enemy > 0)
    {
        p7Enemy[0] /= count7Enemy;
        p7Enemy[1] /= count7Enemy;
    }
    

	// 跨立相交
	// p123Our[2], p456Our[2], p123Enemy[2], p456Enemy[2]
    bool flag = true;
    flag |= cross(p123Our, p456Our, p123Enemy, p456Enemy);
    if ((count7Our > 0) && (count7Enemy > 0))
    {
        flag |= cross(p456Our, p7Our, p456Enemy, p7Enemy);
    }
    return flag;
	
}



/*地图识别*/
void map_identify()
{
	if (map4_identify())
	{
		MAPKIND = 4;
	}
	else if (map2_identify())
	{
		MAPKIND = 2;
	}
	else if (map1_not3_identify())
	{
		MAPKIND = 1;
	}
	else
	{
		MAPKIND = 3;
	}
}

/* 构建expandWall地图 */
void wall_growth(bool cornerExpandSwitch)
{
    //四边界
    for (int i = 0; i < 100; i++)
    {
        expandWall[0][i] = true;
        expandWall[99][i] = true;
        expandWall[i][0] = true;
        expandWall[i][99] = true;
    }

    //左边
    int i = 0;
    for (int j = 1; j < 99; j++)
    {
        if (wallMap[i][j])
        {
            expandWall[i][j] = true;
            expandWall[i + 1][j] = true;
            expandWall[i][j + 1] = true;
            expandWall[i][j - 1] = true;
            if (cornerExpandSwitch == true)
            {
                expandWall[i + 1][j + 1] = true;
                expandWall[i + 1][j - 1] = true;
            }
        }
    }

    //右边
    i = 99;
    for (int j = 1; j < 99; j++)
    {
        if (wallMap[i][j])
        {
            expandWall[i][j] = true;
            expandWall[i - 1][j] = true;
            expandWall[i][j + 1] = true;
            expandWall[i][j - 1] = true;
            if (cornerExpandSwitch == true)
            {
                expandWall[i - 1][j + 1] = true;
                expandWall[i - 1][j - 1] = true;
            }
        }
    }

    //上边
    int j = 0;
    for (int i = 1; i < 99; i++)
    {
        if (wallMap[i][j])
        {
            expandWall[i][j] = true;
            expandWall[i + 1][j] = true;
            expandWall[i - 1][j] = true;
            expandWall[i][j + 1] = true;
            if (cornerExpandSwitch == true)
            {
                expandWall[i + 1][j + 1] = true;
                expandWall[i - 1][j + 1] = true;
            }
        }
    }

    //下边
    j = 99;
    for (int i = 1; i < 99; i++)
    {
        if (wallMap[i][j])
        {
            expandWall[i][j] = true;
            expandWall[i + 1][j] = true;
            expandWall[i - 1][j] = true;
            expandWall[i][j - 1] = true;
            if (cornerExpandSwitch == true)
            {
                expandWall[i + 1][j - 1] = true;
                expandWall[i - 1][j - 1] = true;
            }
        }
    }

    //左上角
    i = 0; j = 0;
    if (wallMap[i][j])
    {
        expandWall[i][j] = true;
        expandWall[i + 1][j] = true;
        expandWall[i][j + 1] = true;
        if (cornerExpandSwitch == true)
        {
            expandWall[i + 1][j + 1] = true;
        }
    }

    //右上角
    i = 99; j = 0;
    if (wallMap[i][j])
    {
        expandWall[i][j] = true;
        expandWall[i - 1][j] = true;
        expandWall[i][j + 1] = true;
        if (cornerExpandSwitch == true)
        {
            expandWall[i - 1][j + 1] = true;
        }
    }

    //左下角
    i = 0; j = 99;
    if (wallMap[i][j])
    {
        expandWall[i][j] = true;
        expandWall[i + 1][j] = true;
        expandWall[i][j - 1] = true;
        if (cornerExpandSwitch == true)
        {
            expandWall[i + 1][j - 1] = true;
        }
    }

    //右下角
    i = 99; j = 99;
    if (wallMap[i][j])
    {
        expandWall[i][j] = true;
        expandWall[i - 1][j] = true;
        expandWall[i][j - 1] = true;
        if (cornerExpandSwitch == true)
        {
            expandWall[i - 1][j - 1] = true;
        }
    }

    //中心区域
    for (int i = 1; i < 99; i++)
    {
        for (int j = 1; j < 99; j++)
        {
            if (wallMap[i][j])
            {
                expandWall[i][j] = true;
                expandWall[i + 1][j] = true;
                expandWall[i - 1][j] = true;
                expandWall[i][j + 1] = true;
                expandWall[i][j - 1] = true;
                if (cornerExpandSwitch == true)
                {
                    expandWall[i + 1][j + 1] = true;
                    expandWall[i - 1][j + 1] = true;
                    expandWall[i + 1][j - 1] = true;
                    expandWall[i - 1][j - 1] = true;
                }
            }
        }
    }
}

/* 构建chessBoard地图 */
void chess_board_creat()
{
    for (int i = 0; i < 101; i++)
    {
        for (int j = 0; j < 101; j++)
        {
            //墙角是i,j
            //对应原坐标上的四个相邻格子是(i,j)(i-1,j)(i,j-1)(i-1,j-1)
            if ((i == 0) || (j == 0) || (i == 100) || (j == 100))
            {
                chessBoard[i][j] = { true };
            }
            else if ((wallMap[i][j] == true) || (wallMap[i - 1][j] == true) || (wallMap[i][j - 1] == true) || (wallMap[i - 1][j - 1] == true))
            {
                chessBoard[i][j] = { true };
            }
        }
    }
}

