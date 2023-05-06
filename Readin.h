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
void init_param();

// 读取初始化地图信息
bool readMapUntilOK();

// 读取帧信息
bool readUntilOK();