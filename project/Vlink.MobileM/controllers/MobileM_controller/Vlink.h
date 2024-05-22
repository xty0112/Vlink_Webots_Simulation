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
// ��е�۲���
constexpr double L2 = 0.135;
constexpr double L1 = 0.15;
constexpr double L0 = 0.069;
constexpr double initx = -0.159335;
constexpr double inity = 0;
constexpr double initz = 0.203916;
constexpr bool right_side = true;
constexpr bool left_side = false;
//flag
constexpr bool left_leg=false;
constexpr bool right_leg=true;
constexpr bool incrementalPID = false;
constexpr bool intengralPID = true;
constexpr int hip_max_T = 10;
constexpr int wheel_max_T = 10;
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
// Ŀǰ���õ�˼·��
// �۲����ĵ�λ�ã���������λ�ã�����theta�������ı仯�ʣ�pid����theta��
// ֱ�ӿ���λ��Ч������ ��֪���ǲ����ҵ����� �켣���ٸо�����ֱ�ӿ����ٶ�
//һЩ�������뷨�� 
//�Ƿ�����ִ�в�ͬ����ʱ�����ò�ͬ��pid��lqr������
//ת�䵽����ʲô���⣨��� lqr��Ȩ�ؾ���R���ֵ����С��
//��theta����X��ʵ�� ������ �Ż��˻�е�۲���ʱ�������Ӽ��ٶ�̬���ܣ�
//��ͬ�ȳ�ʱ��Ҫ�Ŀ��Ʋ�����ͬ�������⣩

//���죺
//�켣����
//��е���˶�ѧ
//�¿��Ʋ��������������ܵļ���
class Vlink : public Robot {
public:
	Vlink();
	void run();
	int count;
	ofstream ofile;
private:
	enum Mode
	{
		CONTROL,PATH,CONNECT,STAND
	}mode;
	int control_state;
	bool m_control_mode;
	bool control_flag = false;
	bool switch_flag=false;
	bool connect_flag = false;
	bool driver_flag = false;
	bool magnet_on = false;
	double switch_angle[4];
	int timecount;
	int timeStep;
	double test=0;
	double D[3], K[3];
	//facilities
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
	double height_exp = 0.20;
	double l0_exp_l = 0.20,l0_exp_r=0.20;
	double manipulator_angle_l[3] = { 0 };
	double manipulator_angle_r[3] = { 0 };
	double exp_angle[3];
	double x, y, z;
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
	ArmModel* Manipulator_R;
	ArmModel* Manipulator_L;
	void kinematic(ArmModel* M);
	void dynamics(ArmModel* M);
	void inverse_kine(ArmModel* M);//exp_pos to exp_angle
	void angle_control(ArmModel* M);//convert DH angle to mechanic angle
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
	void apply_control(legModel* leg);
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
	void COM_print(void);
	void GPS_print(void);
};
#endif
