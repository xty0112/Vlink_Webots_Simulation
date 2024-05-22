#include "manipulator.h"
int main(int argc, char **argv)
{
	Manipulator *manipulator = new Manipulator();
	manipulator->run();
}
void Manipulator::run()
{
	std::ofstream outputfile("err.csv");
	outputfile << "关节1v,关节1err,关节2v,关节2err,关节3v,关节3err\n";
	while (step(timeStep) != -1)
	{

		keyboard();
		sensor(Manipulator_L);
		sensor(Manipulator_R);
		inverse_kine(Manipulator_L);
		inverse_kine(Manipulator_R);
		dynamics(Manipulator_L);
		dynamics(Manipulator_R);
		dynamics_observe(Manipulator_L);
		dynamics_observe(Manipulator_R);
		angle_control(Manipulator_L);
		angle_control(Manipulator_R);

		// cout << "realtou0=" << Manipulator_L->J0->getTorqueFeedback() << endl;
		double err1 = (Manipulator_L->J0->getTorqueFeedback() + Manipulator_L->tou_observe[0]) / Manipulator_L->J0->getTorqueFeedback();
		double err2 = (Manipulator_L->J1->getTorqueFeedback() + Manipulator_L->tou_observe[1]) / Manipulator_L->J1->getTorqueFeedback();
		double err3 = (Manipulator_L->J2->getTorqueFeedback() + Manipulator_L->tou_observe[2]) / Manipulator_L->J2->getTorqueFeedback();
		err1 = 0;
		cout << "err1=" << err1 << endl;
		cout << "err2=" << err2 << endl;
		cout << "err3=" << err3 << endl;
		if (controlflag&&abs(Manipulator_L->angle[1]->get(typeD))<0.1&&abs(Manipulator_L->angle[2]->get(typeD))<0.1&&abs(err2)<1&&abs(err3)<1)
		{
			outputfile << Manipulator_L->angle[0]->get(typeP) << "," << err1 << "," << Manipulator_L->angle[1]->get(typeP) << "," << err2 << "," << Manipulator_L->angle[2]->get(typeP) << "," << err3 << "\n";
		}

		for (size_t i = 0; i < 3; i++)
		{
			// cout << "tou" << i << "=" << Manipulator_L->tou[i] << endl;
			// cout << "exp_angle" << i << "=" << Manipulator_L->exp_angle[i] << endl;
			// cout << "angle" << i << "=" << Manipulator_L->angle[i]->get(typeP) << endl;
		}
		cout << " " << endl;
		for (size_t i = 0; i < 3; i++)
		{
			// cout << "pos" << i << "=" << Manipulator_L->exp_pos[i] << endl;
		}
		cout << "------------" << endl;
	}
}