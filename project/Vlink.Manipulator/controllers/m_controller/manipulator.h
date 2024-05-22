#pragma once
#ifndef MANIPULATOR_H
#define MANIPULATOR_H
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
// ��е�۲���
constexpr double g = 9.8;
constexpr double pi = 3.141592653;
constexpr double L0 = 0.135;
constexpr double L1 = 0.15;
constexpr double L2 = 0.069;
constexpr double initx = -0.159335;
constexpr double inity = 0;
constexpr double initz = 0.203916;
constexpr bool right_side = true;
constexpr bool left_side = false;
using namespace webots;
using namespace std;
enum timetype
{
    typeI, typeP, typeD, typeDD
};
class timevar
{
public:
    timevar(int _timeStep);
    double get(timetype);
    void update(double in);
private:
    double dT;
    double DTS[3];
    double D;
    double I;
    double DD;
};
struct device
{
    bool side;
    Motor* J0, * J1, * J2;
    PositionSensor* J0_sensor, * J1_sensor, * J2_sensor;
    double exp_pos[3] = { 0 };//DH����
    double tou[3] = { 0 };//DH����
    double tou_observe[3] = { 0 };//DH����
    timevar* pos[3] = { 0 };//DH����
    double exp_angle[3] = { 0 };//DH����
    timevar* angle[3];//DH����
};
class Manipulator : public Robot
{
public:
	Manipulator();
    void run();
private:
    bool controlflag;
    int timeStep;
    double x, y, z;
    device* Manipulator_R;
    device* Manipulator_L;
    double D[3], K[3];
    Keyboard* key;
    void exp_set(bool mode, double a0, double a1, double a2);
    void sensor(device* M);
    void keyboard(void);
	void kinematic(device* M);
    void dynamics(device* M);
    void dynamics_observe(device* M);
	void inverse_kine(device* M);//exp_pos to exp_angle
    void angle_control(device* M);//convert DH angle to mechanic angle
};
#endif