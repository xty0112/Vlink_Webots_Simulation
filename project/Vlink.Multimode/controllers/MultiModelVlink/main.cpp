// File:MultiModelVlink
// Date:2024/1/4
// Description:
// Author: Chongshang Yan
// ycs020328@gmail.com
// Modifications
#include "Vlink.h"
//VMC解算出来叫phi的其实是alpha IMU读出来的是phi
/*
*/
void pid_file_read(void);
void lqr_file_read(void);
int main(int argc, char** argv) {
    lqr_file_read();
    pid_file_read();
    Vlink* vlink = new Vlink();
    vlink->run();
    cout << "simulation_end" << endl;
    delete vlink;
    return 0;
}

void Vlink::run() {
    while (step(timeStep) != -1) {
        //获取传感器信息并更新各观测 不改变状态期望值
        observe();
        //施加期望信号
        switch (mode)
        {
        case Vlink::CONTROL:
            keyboard_control_exp_set();
            control();
            if (control_flag)
            {
                //髋关节 轮电机施加力矩控制
                apply_control(lleg);
                apply_control(rleg);
            }
            console_print();
            break;
        case Vlink::PATH:
            path_follow_exp_set(followtest);
            control();
            if (control_flag)
            {
                apply_control(lleg);
                apply_control(rleg);
            }
            break;
        case Vlink::STAND:
            cout<<"press \"R\" to stand up"<<endl;
            keyboard_control_exp_set();
            control();
            break;
        default:
            break;
        }
        //向控制台（或excel文档）输出相关量
        count++;
        if (count>1)
        {
            control_flag = true;
        }
    }
}
/*
转向时，劈叉，不转角度：两腿角度一致控制的pid重新整定（加积分环节）、角度转向惯性大，不能用增量pid
水平转向，转过半圈后突然倒下：期望角度大于pi 或小于-pi后需要取模

左右两腿不对称：可能是复制的时候哪里忘了改，明显错的那条腿公式里面有一部分用的是另一条腿的
机头水平，theta变大 向前倒下：
*/
