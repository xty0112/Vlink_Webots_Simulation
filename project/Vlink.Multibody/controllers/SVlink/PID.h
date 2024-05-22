#pragma once
#ifndef PID_H
#define PID_H
constexpr int pid_num = 5;
extern double pid_index[pid_num * 3];
class PID
{
	//创建对象时调用1初始化 使用时调用2更新反馈值 调用3计算控制输出 调用4设置期望值
public:
	PID(double _kp, double _ki, double _kd, double _exp, bool _mode, int _timeStep);//1
	void DTS_update(double _fdb);//2
	double control_calc(void);//3
	void exp_set(double _exp);//4
	void pid_print();
private:
	bool mode;
	double kp, ki, kd;
	double DTS[3];
	double exp;
	double intengral;
	double derivative;
	double timeStep;
	double control;
}; 
#endif // PID_H
