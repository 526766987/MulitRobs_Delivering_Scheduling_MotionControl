#pragma once

#include<vector>

//敌方探测
vector<vector<float>> enemy_detect();

//工作站角集
extern vector<int> cornerStation;
void corner_station();

// 雷达索敌与危险站确定
void scouting();