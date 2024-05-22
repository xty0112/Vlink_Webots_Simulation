#include "Vlink.h"
manipulator::manipulator() {}
void manipulator::exp_set(bool mode, double a0, double a1, double a2) 
{
	if (mode==true)
	{
		exp[0] += a0;
		exp[1] += a1;
		exp[2] += a2;
	}
	if (mode == false)
	{
		exp[0] = a0;
		exp[1] = a1;
		exp[2] = a2;
	}
}
const double* manipulator::angle_get(void)
{
	angle[0] = J0_sensor->getValue();
	angle[1] = J1_sensor->getValue();
	angle[2] = J2_sensor->getValue();
	return angle;
}
void manipulator::move_exp(void) 
{
	J0->setPosition(exp[0]);
	J1->setPosition(exp[1]);
	J2->setPosition(exp[2]);
}
void manipulator::position_print(void) 
{
	cout << " angle0= " << angle[0] << ", angle1= " << angle[1] << ", angle2= " << angle[2] << endl;
}