#include "manipulator.h"
Manipulator::Manipulator()
{
	timeStep = 8;
	Manipulator_R = (struct device*)malloc(sizeof(struct device));
	Manipulator_L = (struct device*)malloc(sizeof(struct device));
	controlflag = false;
	D[0] = 0.5;
	D[1] = 0.5;
	D[2] = 0.03;
	K[0] = 3;
	K[1] = 10;
	K[2] = 2;
	for (size_t i = 0; i < 3; i++)
	{
		Manipulator_R->angle[i] = new timevar(timeStep);
		Manipulator_L->angle[i] = new timevar(timeStep);
		Manipulator_R->exp_angle[i] = 0;
		Manipulator_L->exp_angle[i] = 0;
		Manipulator_R->pos[i] = new timevar(timeStep);
		Manipulator_L->pos[i] = new timevar(timeStep);
		Manipulator_R->exp_pos[i] = 0;
		Manipulator_L->exp_pos[i] = 0;

	}
	x = initx;
	y = inity;
	z = initz;
	Manipulator_R->side = right_side;
	Manipulator_L->side = left_side;

	Manipulator_R->J0 = getMotor("r0");
	Manipulator_R->J1 = getMotor("r1");
	Manipulator_R->J2 = getMotor("r2");
	Manipulator_R->J0->enableTorqueFeedback(timeStep);
	Manipulator_R->J1->enableTorqueFeedback(timeStep);
	Manipulator_R->J2->enableTorqueFeedback(timeStep);

	Manipulator_R->J0_sensor = getPositionSensor("r0_sensor");
	Manipulator_R->J1_sensor = getPositionSensor("r1_sensor");
	Manipulator_R->J2_sensor = getPositionSensor("r2_sensor");
	Manipulator_R->J0_sensor->enable(timeStep);
	Manipulator_R->J1_sensor->enable(timeStep);
	Manipulator_R->J2_sensor->enable(timeStep);

	Manipulator_L->J0 = getMotor("l0");
	Manipulator_L->J1 = getMotor("l1");
	Manipulator_L->J2 = getMotor("l2");
	Manipulator_L->J0->enableTorqueFeedback(timeStep);
	Manipulator_L->J1->enableTorqueFeedback(timeStep);
	Manipulator_L->J2->enableTorqueFeedback(timeStep);

	Manipulator_L->J0_sensor = getPositionSensor("l0_sensor");
	Manipulator_L->J1_sensor = getPositionSensor("l1_sensor");
	Manipulator_L->J2_sensor = getPositionSensor("l2_sensor");
	Manipulator_L->J0_sensor->enable(timeStep);
	Manipulator_L->J1_sensor->enable(timeStep);
	Manipulator_L->J2_sensor->enable(timeStep);
	key = getKeyboard();
	key->enable(timeStep);
	cout << "manipulator initialized" << endl;
}

void Manipulator::sensor(device *M)
{
	if (M->side==left_side)
	{
		M->angle[0]->update(Manipulator_L->J0_sensor->getValue());
		M->angle[1]->update(pi - Manipulator_L->J1_sensor->getValue());
		M->angle[2]->update(pi - Manipulator_L->J2_sensor->getValue());
	}
	else
	{
		M->angle[0]->update(-Manipulator_R->J0_sensor->getValue());
		M->angle[1]->update(pi + Manipulator_R->J1_sensor->getValue());
		M->angle[2]->update(pi + Manipulator_R->J2_sensor->getValue());
	}
}

void Manipulator::dynamics(device*M)
{
	double theta1 =  M->angle[0]->get(typeP);
	double theta2 =  M->angle[1]->get(typeP);
	if (theta2 < 0)
	{
		theta2 += 2 * pi;
	}
	double theta3 =  M->angle[2]->get(typeP);
	double dtheta1 = M->angle[0]->get(typeD);
	double dtheta2 = M->angle[1]->get(typeD);
	double dtheta3 = M->angle[2]->get(typeD);
	double ddtheta1 = 0;
	double ddtheta2 = 0;
	double ddtheta3 = 0;

	// double ddtheta1 = M->angle[0]->get(typeDD);
	// double ddtheta2 = M->angle[1]->get(typeDD);
	// double ddtheta3 = M->angle[2]->get(typeDD);

	double b_tou1_tmp;
	double b_tou2_tmp;
	double c_tou1_tmp;
	double d_tou1_tmp;
	double e_tou1_tmp;
	double f_tou1_tmp;
	double g_tou1_tmp;
	double h_tou1_tmp;
	double i_tou1_tmp;
	double j_tou1_tmp;
	double tou1_tmp;
	double tou1_tmp_tmp;
	double tou2_tmp;
	tou1_tmp = std::cos(2.0 * theta2);
	b_tou1_tmp = std::sin(2.0 * theta2);
	tou1_tmp_tmp = 2.0 * theta2 - 2.0 * theta3;
	c_tou1_tmp = std::cos(tou1_tmp_tmp);
	d_tou1_tmp = std::sin(tou1_tmp_tmp);
	e_tou1_tmp = std::sin(theta1);
	f_tou1_tmp = std::cos(theta2);
	g_tou1_tmp = std::sin(theta3);
	tou1_tmp_tmp = 2.0 * theta2 - theta3;
	h_tou1_tmp = std::sin(tou1_tmp_tmp);
	i_tou1_tmp = std::cos(theta3);
	j_tou1_tmp = 2.0 * dtheta2 - 2.0 * dtheta3;
	M->tou[0] =
		((7.7037197775489431E-41 * ddtheta1 *
			((((((4.37408213182452E+37 * tou1_tmp -
				2.9855706936575259E+35 * b_tou1_tmp) +
				6.6240727172758063E+37 * i_tou1_tmp) +
				6.6240727172758063E+37 * std::cos(tou1_tmp_tmp)) +
				3.1255679977057712E+37 * c_tou1_tmp) -
				4.5432597512179734E+35 * d_tou1_tmp) +
				7.5866211019107493E+37) -
			7.7037197775489431E-41 * dtheta1 *
			(((((5.9711413873150518E+35 * dtheta2 * tou1_tmp +
				8.74816426364904E+37 * dtheta2 * b_tou1_tmp) +
				6.6240727172758063E+37 * dtheta3 * g_tou1_tmp) +
				4.5432597512179734E+35 * c_tou1_tmp * j_tou1_tmp) +
				6.6240727172758063E+37 * h_tou1_tmp * (2.0 * dtheta2 - dtheta3)) +
				3.1255679977057712E+37 * d_tou1_tmp * j_tou1_tmp)) -
			0.012897148 * g * f_tou1_tmp * e_tou1_tmp) -
		0.252 * g * e_tou1_tmp *
		(0.037121 * std::cos(theta2 - theta3) + 0.15 * f_tou1_tmp);
	tou2_tmp = dtheta1 * dtheta1;
	tou1_tmp_tmp = theta1 + theta2;
	e_tou1_tmp = theta1 - theta2;
	b_tou2_tmp = 3.5E-5 * tou2_tmp * c_tou1_tmp;
	f_tou1_tmp = 0.00240785 * tou2_tmp * d_tou1_tmp;
	j_tou1_tmp = 0.004677246 * g * std::sin(tou1_tmp_tmp - theta3);
	c_tou1_tmp = 0.004677246 * g * std::sin(e_tou1_tmp + theta3);
	M->tou[1] = (((((((((((((0.011590040605512 * ddtheta2 - 0.0043427 * ddtheta3) +
		0.005103 * (dtheta3 * dtheta3) * g_tou1_tmp) +
		b_tou2_tmp) +
		0.005103 * tou2_tmp * h_tou1_tmp) +
		f_tou1_tmp) +
		0.025348574 * g * std::sin(e_tou1_tmp)) -
		0.025348574 * g * std::sin(tou1_tmp_tmp)) -
		j_tou1_tmp) +
		c_tou1_tmp) +
		2.3E-5 * tou2_tmp * tou1_tmp) +
		0.010206 * ddtheta2 * i_tou1_tmp) -
		0.005103 * ddtheta3 * i_tou1_tmp) +
		0.003369670302756 * tou2_tmp * b_tou1_tmp) -
		0.010206 * dtheta2 * dtheta3 * g_tou1_tmp;
	tou1_tmp_tmp = 0.0025515 * tou2_tmp;
	M->tou[2] = ((((((((0.0048427 * ddtheta3 - 0.0043427 * ddtheta2) +
		tou1_tmp_tmp * g_tou1_tmp) +
		0.005103 * (dtheta2 * dtheta2) * g_tou1_tmp) -
		b_tou2_tmp) -
		tou1_tmp_tmp * h_tou1_tmp) -
		f_tou1_tmp) +
		j_tou1_tmp) -
		c_tou1_tmp) -
		0.005103 * ddtheta2 * i_tou1_tmp;
	for (size_t i = 0; i < 3; i++)
	{
		M->tou[i] += D[i]*(-M->angle[i]->get(typeD))+K[i]*(M->exp_angle[i]-M->angle[i]->get(typeP));
	}
}
void Manipulator::dynamics_observe(device*M)
{
	double theta1 =  M->angle[0]->get(typeP);
	double theta2 =  M->angle[1]->get(typeP);
	if (theta2 < 0)
	{
		theta2 += 2 * pi;
	}
	double theta3 =  M->angle[2]->get(typeP);
	double dtheta1 = M->angle[0]->get(typeD);
	double dtheta2 = M->angle[1]->get(typeD);
	double dtheta3 = M->angle[2]->get(typeD);

	double ddtheta1 = M->angle[0]->get(typeDD);
	double ddtheta2 = M->angle[1]->get(typeDD);
	double ddtheta3 = M->angle[2]->get(typeDD);

	double b_tou1_tmp;
	double b_tou2_tmp;
	double c_tou1_tmp;
	double d_tou1_tmp;
	double e_tou1_tmp;
	double f_tou1_tmp;
	double g_tou1_tmp;
	double h_tou1_tmp;
	double i_tou1_tmp;
	double j_tou1_tmp;
	double tou1_tmp;
	double tou1_tmp_tmp;
	double tou2_tmp;
	tou1_tmp = std::cos(2.0 * theta2);
	b_tou1_tmp = std::sin(2.0 * theta2);
	tou1_tmp_tmp = 2.0 * theta2 - 2.0 * theta3;
	c_tou1_tmp = std::cos(tou1_tmp_tmp);
	d_tou1_tmp = std::sin(tou1_tmp_tmp);
	e_tou1_tmp = std::sin(theta1);
	f_tou1_tmp = std::cos(theta2);
	g_tou1_tmp = std::sin(theta3);
	tou1_tmp_tmp = 2.0 * theta2 - theta3;
	h_tou1_tmp = std::sin(tou1_tmp_tmp);
	i_tou1_tmp = std::cos(theta3);
	j_tou1_tmp = 2.0 * dtheta2 - 2.0 * dtheta3;
	M->tou_observe[0] =
		((7.7037197775489431E-41 * ddtheta1 *
			((((((4.37408213182452E+37 * tou1_tmp -
				2.9855706936575259E+35 * b_tou1_tmp) +
				6.6240727172758063E+37 * i_tou1_tmp) +
				6.6240727172758063E+37 * std::cos(tou1_tmp_tmp)) +
				3.1255679977057712E+37 * c_tou1_tmp) -
				4.5432597512179734E+35 * d_tou1_tmp) +
				7.5866211019107493E+37) -
			7.7037197775489431E-41 * dtheta1 *
			(((((5.9711413873150518E+35 * dtheta2 * tou1_tmp +
				8.74816426364904E+37 * dtheta2 * b_tou1_tmp) +
				6.6240727172758063E+37 * dtheta3 * g_tou1_tmp) +
				4.5432597512179734E+35 * c_tou1_tmp * j_tou1_tmp) +
				6.6240727172758063E+37 * h_tou1_tmp * (2.0 * dtheta2 - dtheta3)) +
				3.1255679977057712E+37 * d_tou1_tmp * j_tou1_tmp)) -
			0.012897148 * g * f_tou1_tmp * e_tou1_tmp) -
		0.252 * g * e_tou1_tmp *
		(0.037121 * std::cos(theta2 - theta3) + 0.15 * f_tou1_tmp);
	tou2_tmp = dtheta1 * dtheta1;
	tou1_tmp_tmp = theta1 + theta2;
	e_tou1_tmp = theta1 - theta2;
	b_tou2_tmp = 3.5E-5 * tou2_tmp * c_tou1_tmp;
	f_tou1_tmp = 0.00240785 * tou2_tmp * d_tou1_tmp;
	j_tou1_tmp = 0.004677246 * g * std::sin(tou1_tmp_tmp - theta3);
	c_tou1_tmp = 0.004677246 * g * std::sin(e_tou1_tmp + theta3);
	M->tou_observe[1] = (((((((((((((0.011590040605512 * ddtheta2 - 0.0043427 * ddtheta3) +
		0.005103 * (dtheta3 * dtheta3) * g_tou1_tmp) +
		b_tou2_tmp) +
		0.005103 * tou2_tmp * h_tou1_tmp) +
		f_tou1_tmp) +
		0.025348574 * g * std::sin(e_tou1_tmp)) -
		0.025348574 * g * std::sin(tou1_tmp_tmp)) -
		j_tou1_tmp) +
		c_tou1_tmp) +
		2.3E-5 * tou2_tmp * tou1_tmp) +
		0.010206 * ddtheta2 * i_tou1_tmp) -
		0.005103 * ddtheta3 * i_tou1_tmp) +
		0.003369670302756 * tou2_tmp * b_tou1_tmp) -
		0.010206 * dtheta2 * dtheta3 * g_tou1_tmp;
	tou1_tmp_tmp = 0.0025515 * tou2_tmp;
	M->tou_observe[2] = ((((((((0.0048427 * ddtheta3 - 0.0043427 * ddtheta2) +
		tou1_tmp_tmp * g_tou1_tmp) +
		0.005103 * (dtheta2 * dtheta2) * g_tou1_tmp) -
		b_tou2_tmp) -
		tou1_tmp_tmp * h_tou1_tmp) -
		f_tou1_tmp) +
		j_tou1_tmp) -
		c_tou1_tmp) -
		0.005103 * ddtheta2 * i_tou1_tmp;
}
void Manipulator::inverse_kine(device* M)
{
	double X = M->exp_pos[0];
	double Y = M->exp_pos[1];
	double Z = M->exp_pos[2];
	double T1;
	[[maybe_unused]] double T2_1, T3_1, T2_2, T3_2;
	double c_R_tmp[16];
	double temp[4];
	double R_tmp;
	double b_R_tmp;
	T1 = std::atan(Y / X);
	R_tmp = std::sin(T1);
	b_R_tmp = std::cos(T1);
	c_R_tmp[0] = b_R_tmp;
	c_R_tmp[4] = R_tmp;
	c_R_tmp[8] = 0.0;
	c_R_tmp[12] = 0.0;
	c_R_tmp[1] = -R_tmp;
	c_R_tmp[5] = b_R_tmp;
	c_R_tmp[9] = 0.0;
	c_R_tmp[13] = 0.0;
	c_R_tmp[2] = 0.0;
	c_R_tmp[6] = 0.0;
	c_R_tmp[10] = 1.0;
	c_R_tmp[14] = -L2;
	c_R_tmp[3] = 0.0;
	c_R_tmp[7] = 0.0;
	c_R_tmp[11] = 0.0;
	c_R_tmp[15] = 1.0;
	for (int i{ 0 }; i < 4; i++) {
		temp[i] = ((c_R_tmp[i] * X + c_R_tmp[i + 4] * Y) + c_R_tmp[i + 8] * Z) +
			c_R_tmp[i + 12];
	}
	double T2_1_tmp;
	double b_T2_1_tmp;
	double c_T2_1_tmp;
	double d_T2_1_tmp;
	double e_T2_1_tmp;
	double f_T2_1_tmp;
	double g_T2_1_tmp;
	double h_T2_1_tmp;
	double i_T2_1_tmp;
	double j_T2_1_tmp;
	T2_1_tmp = L0 * L0;
	b_T2_1_tmp = 2.0 * L0 * L1;
	R_tmp = L1 * L1;
	b_R_tmp = temp[0] * temp[0];
	c_T2_1_tmp = temp[2] * temp[2];
	d_T2_1_tmp = (((-T2_1_tmp + b_T2_1_tmp) - R_tmp) + b_R_tmp) + c_T2_1_tmp;
	e_T2_1_tmp =
		std::sqrt(d_T2_1_tmp *
			((((T2_1_tmp + b_T2_1_tmp) + R_tmp) - b_R_tmp) - c_T2_1_tmp));
	f_T2_1_tmp = 2.0 * L1 * temp[2];
	g_T2_1_tmp = T2_1_tmp * e_T2_1_tmp / d_T2_1_tmp;
	h_T2_1_tmp = R_tmp * e_T2_1_tmp / d_T2_1_tmp;
	i_T2_1_tmp = b_R_tmp * e_T2_1_tmp / d_T2_1_tmp;
	j_T2_1_tmp = c_T2_1_tmp * e_T2_1_tmp / d_T2_1_tmp;
	b_T2_1_tmp = b_T2_1_tmp * e_T2_1_tmp / d_T2_1_tmp;
	T2_1_tmp =
		(((-T2_1_tmp + R_tmp) + 2.0 * L1 * temp[0]) + b_R_tmp) + c_T2_1_tmp;
	T2_1 = 2.0 *
		std::atan((((((f_T2_1_tmp + g_T2_1_tmp) + h_T2_1_tmp) - i_T2_1_tmp) -
			j_T2_1_tmp) -
			b_T2_1_tmp) /
			T2_1_tmp);//atan((F + E) / G)
	T2_2 = 2.0 *
		std::atan((((((f_T2_1_tmp - g_T2_1_tmp) - h_T2_1_tmp) + i_T2_1_tmp) +
			j_T2_1_tmp) +
			b_T2_1_tmp) /
			T2_1_tmp);//atan((F - E) / G)
	R_tmp = std::atan(e_T2_1_tmp / d_T2_1_tmp);
	T3_1 = -2.0 * R_tmp;
	T3_2 = 2.0 * R_tmp;
	M->exp_angle[0] = T1;
	M->exp_angle[1] = T2_2;
	M->exp_angle[2] = T3_2;
}

void Manipulator::keyboard()
{
	int k = key->getKey();
	switch (k)
	{
	case 'P':
		controlflag = true;
		break;
	case 'T':
		controlflag = false;
		break;
	case 'Z':
		x = initx;
		y = inity;
		z = initz;
		break;
	case '0':
		x = initx;
		y = -0.033;
		z = initz;
		break;
	case '1':
		x = -0.139335;
		y = -0.033;
		z = 0.272916;
		break;
	case '2':
		x = -0.139335;
		y = -0.005;
		z = 0.272916;
		break;
	case '3':
		x = -0.039335;
		y = -0.005;
		z = 0.272916;
		break;
	case '4':
		x = -0.039335;
		y = -0.033;
		z = 0.272916;
		break;
	case 'Q':
		x -= 0.001;
		break;
	case 'E':
		x += 0.001;
		break;
	case 'A':
		y -= 0.001;
		break;
	case 'D':
		y += 0.001;
		break;
	case 'S':
		z -= 0.001;
		break;
	case 'W':
		z += 0.001;
		break;
	}
	Manipulator_L->exp_pos[0] = x;
	Manipulator_L->exp_pos[1] = y;
	Manipulator_L->exp_pos[2] = z;
	Manipulator_R->exp_pos[0] = x;
	Manipulator_R->exp_pos[1] = y;
	Manipulator_R->exp_pos[2] = z;
}
constexpr double Limit=2;
double protect(double _in)
{
	double _output;
	if (_in>Limit)
	{
		_output = Limit;
	}
	if (_in<-Limit)
	{
		_output = -Limit;
	}
	if (_in>-Limit&&_in<Limit)
	{
		_output = _in;
	}
	return _output;
}
void Manipulator::angle_control(device* M)
{
	if (M->exp_angle[1] < 0)
	{
		M->exp_angle[1] += 2 * pi;
	}
	double angle0 = M->exp_angle[0];
	double angle1 = pi - M->exp_angle[1];
	double angle2 = pi - M->exp_angle[2];
	double torq0 = M->tou[0];
	double torq1 = -M->tou[1];
	double torq2 = -M->tou[2];
	if (M->side==right_side)
	{
		angle0 = -angle0;
		angle1 = -angle1;
		angle2 = -angle2;
		torq0 = -torq0;
		torq1 = -torq1;
		torq2 = -torq2;
	}
	if (controlflag)
	{
		cout << "Position" << endl;
		M->J0->setPosition(angle0);
		M->J1->setPosition(angle1);
		M->J2->setPosition(angle2);
	}
	else
	{
		cout << "Torque" << endl;
		M->J0->setTorque(protect(torq0));
		M->J1->setTorque(protect(torq1));
		M->J2->setTorque(protect(torq2));
	}
}