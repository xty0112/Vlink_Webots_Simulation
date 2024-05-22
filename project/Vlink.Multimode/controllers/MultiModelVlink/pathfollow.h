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
	//·����㣬�᲻�ϱ仯Ϊ��һ��Ҫǰ���ĵ�
	struct Path* start;
	//��������λ��
	struct Path* now;
	//�Ӿ����
	double K, Con;
	double ld;
	//������Ϣ
	double exp_velocity;
	double test;
};
#endif // !1

