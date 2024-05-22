#pragma once
#ifndef DEVICE_H
#define DEVICE_H
#include"timeFunc.h"
using namespace webots;
struct legModel
{
	bool flag;
	//leg
	timeFunc* phi_b, * phi_f;//real model,hip motor angle
	double l0;//Virtual model,length of leg
	timeFunc* dl0;//Virtual model,d(length of leg)
	double phi0;
	double dphi0;
	double FN;
	//foot
	timeFunc* foot_angle;
	double last_dx;
	//control output
	double Torque_f, Torque_b;
	double T_foot;
	//VMC input:
	double Tp;//Tp is the output torque of virtual hip motor
	double F;//F is the force along the link
	//leg_state:
	double lqr_line1[6] = { 0 };
	double lqr_line2[6] = { 0 };
	timeFunc* dtheta;
	double state[6] = { 0 };
	double exp_state[6] = { 0 };
	//leg device
	Motor* hip_f, * hip_b;
	Motor* wheel;
	PositionSensor* hipSensor_f, * hipSensor_b;
	PositionSensor* footSensor;
};
struct ArmModel
{
	bool side;
	Motor* J0, * J1, * J2;
	PositionSensor* J0_sensor, * J1_sensor, * J2_sensor;
	double exp_pos[3] = { 0 };//DH定义
	double tou[3] = { 0 };//DH定义
	double pos[3] = { 0 };//DH定义
	double exp_angle[3] = { 0 };//DH定义
	timeFunc* angle[3];//DH定义
};
#endif // !DEVICE_H

