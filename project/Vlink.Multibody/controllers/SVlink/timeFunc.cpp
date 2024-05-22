#include "Vlink.h"
timeFunc::timeFunc(int timeStep)
{
    Dt = (double)timeStep / 1000;
    DTS[2] = 0;
    DTS[1] = 0;
    DTS[0] = 0;
    i = 0;
}
void timeFunc::update(double in) 
{
    DTS[2] = DTS[1];
    DTS[1] = DTS[0];
    DTS[0] = in;
    last_d = d;
    d = (DTS[0] - DTS[1]) / Dt;
    dd = (d - last_d) / Dt;
    i += (DTS[0] + DTS[1]) * Dt * 0.5;
}
void timeFunc::clear_intenger(void)
{
    i = 0;
}
double timeFunc::now(void)
{
    return DTS[0];
}


