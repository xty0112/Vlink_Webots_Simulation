#include "manipulator.h"
timevar::timevar(int _timeStep)
{
	dT = (double)_timeStep / 1000;
    DTS[0]=0;
    DTS[1] = 0;
    DTS[2] = 0;
    D=0;
    I=0;
    DD=0;
}
void timevar::update(double _in)
{
    DTS[2] = DTS[1];
    DTS[1] = DTS[0];
    DTS[0] = _in;
    D = (DTS[0] - DTS[1]) / dT;
    DD = (DTS[0] - 2 * DTS[1] + DTS[2]) / dT / dT;
    I += (DTS[0] + DTS[1]) / 2 * dT;
}
double timevar::get(timetype TT)
{
    double output=0;
    if (TT == typeP)
    {
        output = DTS[0];
    }
    if (TT == typeI)
    {
        output = I;
    }
    if (TT == typeD)
    {
        output = D;
    }
    if (TT == typeDD)
    {
        output = DD;
    }
    return output;
}