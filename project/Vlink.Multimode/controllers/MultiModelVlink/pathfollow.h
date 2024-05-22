#pragma once
#ifndef PATHFOLLOW_H
#define PATHFOLLOW_H
struct Path
{
	bool state;
	double x;
	double z;
	double angle;
	double dx;
	double dz;
	Path* next;
};
const int pointnum = 20;
extern Path* p[pointnum];
class PathFollow
{
public:
	PathFollow(Path* _start, double _K, double _Con);
	void update_car_info(double x, double z, double dx, double dz, double pitch, double exp_velocity);
	void set_path(Path* _start);
	double angle_out(void);
	double exp_out(void);
	Path* goal_out(void);
private:
	void pid_follow();
	//路径起点，会不断变化为下一个要前往的点
	struct Path* start;
	//车辆现在位置
	struct Path* now;
	//视距参数
	double K, Con;
	double ld;
	//车辆信息
	double exp_velocity;
	double test;
};
#endif // !1

