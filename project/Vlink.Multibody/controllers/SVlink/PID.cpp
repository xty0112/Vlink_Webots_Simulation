#include "Vlink.h"
PID::PID(double _kp, double _Ti, double _Td, double _exp, bool _mode, int _timeStep)
{
    mode = _mode;
    exp = _exp;
    DTS[0] = 0;
    DTS[1] = 0;
    DTS[2] = 0;
    timeStep = (double)_timeStep/1000;
    control = 0;
    kp = _kp;
    ki = _Ti;
    kd = _Td;
}
void PID::DTS_update(double fdb)
{
    double err;
    err = exp - fdb;
    DTS[2] = DTS[1];
    DTS[1] = DTS[0];
    DTS[0] = err;
    intengral += err;
    derivative = (DTS[0] - DTS[1]) / timeStep;
    if (mode == incrementalPID)
    {
        control += kp * (DTS[0] - DTS[1])
            + ki * DTS[0]
            + kd * (DTS[0] - 2 * DTS[1] + DTS[2]);
    }
    else
    {
        control = kp * DTS[0]
            + ki * intengral
            + kd * (DTS[0] - 2 * DTS[1] + DTS[2]);
    }
}
double PID::control_calc(void)
{
    return control;
}
void PID::exp_set(double _exp)
{
    exp = _exp;
}