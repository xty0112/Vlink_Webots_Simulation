#pragma once
#ifndef TIMEFUNC_H
#define TIMEFUNC_H
class timeFunc
{
public:
	timeFunc(int timeStep);
	double now(void);
	void update(double);
	void clear_intenger(void);
	double d;
	double last_d;
	double dd;
	double i;
private:
	double DTS[3];
	double Dt;
};
#endif // !TIMEFUNC_H

