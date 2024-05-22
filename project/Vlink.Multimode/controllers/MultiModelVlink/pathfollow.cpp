#include "Vlink.h"
double distance_calc(Path* A, Path* B) {
	return sqrt((A->x - B->x) * (A->x - B->x) + (A->z - B->z) * (A->z - B->z));
}
struct Path* transfer(double co_x, double co_z, double rotate, double point_x, double point_z)
{
	//测试看的时候GPS俯视不是右手系，给Z轴反转一下
	Path* temp=(struct Path*)malloc(sizeof(struct Path));//wtf
	temp->x = (point_x - co_x) * cos(rotate) + (co_z - point_z) * sin(rotate);
	temp->z = (co_z - point_z) * cos(rotate) - (point_x - co_x) * sin(rotate);
	return temp;
}
PathFollow::PathFollow(struct Path* _start, double _K, double _Con)
{
	start = _start;
	K = _K;
	Con = _Con;
	now = (struct Path*)malloc(sizeof(struct Path));
	
}
void PathFollow::set_path(Path* _start)
{
	start = _start;
}
void PathFollow::update_car_info(double _x, double _z,double _dx,double _dz,double _pitch,double _exp_velocity)
{
	now->x = _x;
	now->z = _z;
	now->dx = _dx;
	now->dz = _dz;
	now->angle = _pitch;
	exp_velocity = _exp_velocity * pow(1 / 2.7, angle_out() * angle_out()*0.25);
	pid_follow();
}
double PathFollow::angle_out()
{
	return test;
}
double PathFollow::exp_out()
{
	return exp_velocity;
}
Path* PathFollow::goal_out()
{
	return start;
}
void PathFollow::pid_follow() 
{
	//找到前进方向最近的一个节点
	double distance,close_d;
	Path* temp = start;
	Path* close_point = NULL;
	close_d = 999;
	ld = K * sqrt(now->dx * now->dx + now->dz * now->dz)+Con;
	while(temp != NULL)
	{
		distance = distance_calc(now,temp);
		//cout << "distance=" << distance << endl;
		if (distance<close_d&&distance>=ld)
		{
			close_point = temp;
			close_d = distance;
		}
		temp = temp->next;
		//cout << "temp=" << temp->x << endl;
	}
	start = close_point;
	if (start!=NULL)
	{
	cout << "goal_point x=" << start->x << ",z=" << start->z << endl;
	struct Path* next_in_car_co = (struct Path*)malloc(sizeof(struct Path));
	next_in_car_co = transfer(now->x, now->z, now->angle, start->x, start->z);
	test = atan(next_in_car_co->z / next_in_car_co->x);
	}
}
double path1(double x)
{
	//return 2 * (cos(0.1 * x * pi) - 1);
	return 2 * (cos(0.2 * x * pi) - 1);
}
Path* p[pointnum];
Path* path_generate()
{
	double tempx = 0;
	double tempz = 0;
	p[0] = (struct Path*)malloc(sizeof(struct Path));
	p[0]->x = 0;
	p[0]->z = 0;
	for (size_t i = 0; i < pointnum - 2; i++)
	{
		p[i + 1] = (struct Path*)malloc(sizeof(struct Path));
		tempx += 0.5;
		tempz = path1(tempx);
		p[i]->next = p[i + 1];
		p[i + 1]->x = tempx;
		p[i + 1]->z = tempz;
	}
	return p[0];
}