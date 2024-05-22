#pragma once
#ifndef VLINK_H
#define VLINK_H

constexpr double pi=3.1416;
constexpr double g = 9.8;
//link length
constexpr double l1=0.14;
constexpr double l2=0.24;
constexpr double l3=0.24;
constexpr double l4=0.14;
constexpr double l5=0.095;
constexpr double radius = 0.053;
constexpr double half_wheel_d = 0.2955 / 2;
//link weight
constexpr double m_wheel = 0.5219;
//flag
constexpr bool left_leg=false;
constexpr bool right_leg=true;
constexpr bool incrementalPID = false;
constexpr bool intengralPID = true;
constexpr int max_T = 10;
//extern variable
extern double lqr_index[12][4];
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Connector.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/utils/AnsiCodes.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <fstream>

#include "PID.h"
#include "pathfollow.h"
#include "timeFunc.h"
#include "device.h"

using namespace webots;
using namespace std;
// 目前采用的思路：
// 观测重心的位置，利用重心位置（折算theta）和重心变化率（pid折算theta）
// 直接控制位置效果很拉 不知道是不是我的问题 轨迹跟踪感觉还是直接控制速度
//一些其他的想法： 
//是否考虑在执行不同动作时，采用不同的pid或lqr参数？
//转弯到底是什么问题（解决 lqr中权重矩阵R，轮电机调小）
//用theta控制X（实现 待兼容 优化了机械臂不动时的整机加减速动态性能）
//不同腿长时需要的控制参数不同（大问题）

//待办：
//轨迹跟踪
//机械臂运动学
//新控制补偿器与其他功能的兼容
class manipulator;
class Vlink : public Robot {
public:
	Vlink();
	void run();
	int count;
	ofstream ofile;
private:
	enum Mode
	{
		CONTROL,PATH,CONNECT
	}mode;
	int control_state;
	bool control_flag = false;
	bool switch_flag=false;
	bool connect_flag = false;
	bool driver_flag = false;
	bool magnet_on = false;
	double switch_angle[4];
	int timecount;
	int timeStep;
	//facilities
	Receiver* receiver;
	Connector* armature_l;
	Connector* armature_r;
	Connector* magnet_l;
	Connector* magnet_r;
	Gyro* gyro;
	InertialUnit* imu;
	Keyboard* key;
	Accelerometer* ACC;
	GPS* gps;
	PathFollow* followtest;
	//center of mass
	timeFunc* COM;
	double COM_proj;

	//expectations
	double height_exp = 0.18;
	double l0_exp_l = 0.18,l0_exp_r=0.18;
	double manipulator_angle_l[3] = { 0 };
	double manipulator_angle_r[3] = { 0 };
	double exp_angle[3];

	//PIDs
	PID* test_pid;
	PID* l0_pid_l;
	PID* l0_pid_r;//use for leg_length
	PID* theta_pid;//use for leg_consistency
	PID* turn_pid;//use for turn
	PID* roll_pid;
	//store sensor output
    double angle[3];//from imu
	double dangle[3] = { 0 };//from gyro
	double ddx[3] = { 0 };//from acc
	timeFunc* gps_pos[3];//from gps
	
	//parts	
	legModel LLL, RRR;
	legModel *lleg=&LLL,*rleg=&RRR;
	manipulator* Manipulator_R;
	manipulator* Manipulator_L;

	//observe.cpp
	void observe(void);
	void sensor_get(void);
	void COM_calc(void);
	void leg_pos(legModel* leg);
	void leg_spd(legModel* leg);
	void leg_conv(legModel* leg);
	void FN_solve(legModel*leg);
	//LQR.cpp
	void lqr_fdb(legModel* leg);
	void lqr_k1(legModel* leg);
	//control.cpp
	void control(void);
	void COM_comp(void);
	void keyboard_control_exp_set(void);
	void path_follow_exp_set(PathFollow* pathfollow);
	void turn_control(void);
	void leg_length_control(void);
	void apply_control(legModel* leg,bool mode);
	//extern_functions.cpp
	void file_print(double a1, double a2, double a3, double a4, double a5, double a6);
	//test_functions.cpp
	void console_print(void);
	void Torque_print(void);
	void state_print(bool f1,bool f2,bool f3);
	void virtual_model_print();
	void leg_print(legModel* leg);
	void pid_print(PID* _pid);
	void odo_print(void);
	void imu_print(void);
	void lqr_print(void);
	void manipulator_print(manipulator* M);
	void COM_print(void);
	void GPS_print(void);
};
#endif
