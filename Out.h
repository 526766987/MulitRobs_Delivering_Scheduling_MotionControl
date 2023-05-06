#pragma once
#include <fstream>
#include "Global.h"

using namespace std;

extern ofstream outfile0; //蓝队输出文件流
extern ofstream outfile1; //红队输出文件流

/*输出调试信息到文件 */
template<typename... Args>
static void outdebug(const Args&... args) {
    if(!outfile0.is_open()) {
		outfile0.open(FILE0);
	}
    if(!outfile1.is_open()) {
		outfile1.open(FILE1);
	}
    if(TEAM == 0) {
        outfile0 << "[BLUE TEAM] ";
        outfile0 << state.timeStamp << " : ";
        ((outfile0 << args << " "), ...);
        outfile0 << endl;
    } else if(TEAM == 1) {
        outfile1 << "[RED TEAM]  ";
        outfile1 << state.timeStamp << " : ";
        ((outfile1 << args << " "), ...);
        outfile1 << endl;
    }
}

/*输出调试信息到文件不换行 */
template<typename... Args>
static void outdebugline(const Args&... args) {
    if(!outfile0.is_open()) {
		outfile0.open(FILE0);
	}
    if(!outfile1.is_open()) {
		outfile1.open(FILE1);
	}
	((outfile1 << args << " "), ...);
}

/* 输出线速度调整指令 */
void send_cmd_forward(int robot_id, float speed);

/* 输出角速度调整指令 */
void send_cmd_rotate(int robot_id, float speed);

/* 输出购买指令 */
void send_cmd_buy(int robot_id);

/* 输出出售指令 */
void send_cmd_sell(int robot_id);

/* 输出销毁指令 */
void send_cmd_destroy(int robot_id);