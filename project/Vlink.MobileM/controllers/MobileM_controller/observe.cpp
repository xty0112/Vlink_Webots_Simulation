#include "Vlink.h"
void Vlink::observe(void) {
	//�۲� �����ĸ����ȱ�ʵ�ʽǶ�phi[0~3]���ٶȼ����
	sensor_get();
	//����phi0(VMC���������������н� ������),l0(����˳��� ������),����״̬�ռ� ��ǰ�����еĵ���
	leg_pos(lleg);
	leg_pos(rleg);
	leg_spd(lleg);
	leg_spd(rleg);
	followtest->update_car_info(gps_pos[0]->now(), gps_pos[2]->now(), gps_pos[0]->d, gps_pos[2]->d, angle[2], 1);
}
void Vlink::sensor_get(void) {
	const double* temp1;
	const double* temp2;
	const double* temp3;
	//const double* temp4;
	//const double* temp5;
	const double* temp6;
	//IMU&GYRO
	for (size_t i = 0; i < 3; i++)
	{
		temp1 = &imu->getRollPitchYaw()[i];
		temp2 = &gyro->getValues()[i];
		temp3 = &ACC->getValues()[i];
		/*
		temp4 = &Manipulator_L->angle_get()[i];
		temp5 = &Manipulator_R->angle_get()[i];
		*/
		temp6 = &gps->getValues()[i];
		angle[i] = *temp1;
		dangle[i] = *temp2;
		ddx[i] = *temp3;
		/*
		manipulator_angle_l[i] = *temp4;
		manipulator_angle_r[i] = *temp5;
		*/
		gps_pos[i]->update(*temp6);
	}
	angle[0] += 0.5 * pi;
	//foot motor
	lleg->foot_angle->update(lleg->footSensor->getValue());
	rleg->foot_angle->update(rleg->footSensor->getValue());
	//hip motor
	lleg->phi_f->update(-lleg->hipSensor_f->getValue());
	rleg->phi_f->update(-rleg->hipSensor_f->getValue());
	lleg->phi_b->update(pi - lleg->hipSensor_b->getValue());
	rleg->phi_b->update(pi - rleg->hipSensor_b->getValue());
	//arm motor
	Manipulator_L->angle[0]->update(Manipulator_L->J0_sensor->getValue());
	Manipulator_L->angle[1]->update(pi - Manipulator_L->J1_sensor->getValue());
	Manipulator_L->angle[2]->update(pi - Manipulator_L->J2_sensor->getValue());
	Manipulator_R->angle[0]->update(-Manipulator_R->J0_sensor->getValue());
	Manipulator_R->angle[1]->update(pi + Manipulator_R->J1_sensor->getValue());
	Manipulator_R->angle[2]->update(pi + Manipulator_R->J2_sensor->getValue());
}
void Vlink::COM_calc(void) 
{	
	double t0=0, t1=0, t2=0;
	// t0 = Manipulator_L->angle[0];
	// t1 = Manipulator_L->angle[1];
	// t2 = Manipulator_L->angle[2];
	COM->update(0.01*(1940219 / 14830 - (250 * cos(t0) * ((58464 * cos(t1 - t2)) / 3125 - (253493 * cos(t1)) / 2500)) / 1483));
	COM_proj=(COM->now() - 0.95)* cos(lleg->state[4]);
}
void Vlink::leg_conv(struct legModel* leg) {
	double T_tmp;
	double T_tmp_tmp;
	double a;
	double a_tmp;
	double b_T_tmp;
	double b_a_tmp;
	double c_T_tmp;
	double c_a_tmp;
	double t10;
	double t11;
	double t119;
	double t122;
	double t23;
	double t24;
	double t25;
	double t26;
	double t40;
	double t41;
	double t42;
	double t48;
	double t49;
	double t56_tmp;
	double t6;
	double t73_tmp;
	double t74_tmp;
	double t8;
	double t84_tmp;
	double t9;
	double t94;
	double t97;
	//     This function was generated by the Symbolic Math Toolbox version 9.2.
	//     26-Jul-2023 19:13:29
	t6 = l2 * l2;
	t8 = l1 * std::cos(leg->phi_b->now());
	t9 = l4 * std::cos(leg->phi_f->now());
	t10 = l1 * std::sin(leg->phi_b->now());
	t11 = l4 * std::sin(leg->phi_f->now());
	t23 = t10 - t11;
	t24 = (l5 + t9) - t8;
	t25 = t23 * t23;
	t26 = t24 * t24;
	t40 = t8 * t23 * 2.0 + t10 * t24 * 2.0;
	t41 = t9 * t23 * 2.0 + t11 * t24 * 2.0;
	t42 = ((t6 - l3 * l3) + t25) + t26;
	t48 = 1.0 / (l2 * t24 * 2.0 + t42);
	t49 = t48 * t48;
	t25 = std::sqrt((t6 * t25 * 4.0 + t6 * t26 * 4.0) - t42 * t42);
	t56_tmp = 1.0 / t25;
	a_tmp = l2 * t23 * 2.0 - t25;
	t25 = std::atan(-t48 * a_tmp) * 2.0;
	t73_tmp = std::cos(t25);
	t74_tmp = std::sin(t25);
	t84_tmp = 1.0 / (t49 * (a_tmp * a_tmp) + 1.0);
	t25 = l2 * t73_tmp;
	t26 = l2 * t74_tmp;
	a = t10 + t26;
	b_a_tmp = t8 - l5 / 2.0;
	c_a_tmp = b_a_tmp + t25;
	t94 = t10 + t26;
	t97 = b_a_tmp + t25;
	t119 = (t10 + l2 * 0.0) + t26;
	b_a_tmp = c_a_tmp - l2 * 0.0;
	t122 = b_a_tmp * b_a_tmp;
	T_tmp_tmp =
		-(l2 * t10 * 2.0 + t40) * t49 * a_tmp +
		t48 * (l2 * t8 * 2.0 - t56_tmp *
			((t6 * t8 * t23 * 8.0 + t6 * t10 * t24 * 8.0) -
				t40 * t42 * 2.0) /
			2.0);
	T_tmp = l2 * 0.0 * 2.0;
	b_T_tmp = t25 * t84_tmp;
	c_T_tmp = t26 * t84_tmp;
	c_a_tmp = leg->F * (1.0 / std::sqrt(a * a + c_a_tmp * c_a_tmp));
	a = -t73_tmp * t84_tmp;
	t26 = -t74_tmp * t84_tmp;
	t40 = t119 * (1.0 / t122);
	t25 = leg->Tp * t122 * (1.0 / (t119 * t119 + t122));
	leg->Torque_f = (c_a_tmp *
		(t94 * (t8 - b_T_tmp * T_tmp_tmp * 2.0) * 2.0 -
			t97 * (t10 - c_T_tmp * T_tmp_tmp * 2.0) * 2.0) /
		2.0 +
		t25 * (((t8 - T_tmp) + l2 * (a * T_tmp_tmp) * 2.0) / b_a_tmp +
			t40 * ((t10 + T_tmp) + l2 * (t26 * T_tmp_tmp) * 2.0)));
	T_tmp_tmp =
		-(l2 * t11 * 2.0 + t41) * t49 * a_tmp +
		t48 * (l2 * t9 * 2.0 - t56_tmp *
			((t6 * t9 * t23 * 8.0 + t6 * t11 * t24 * 8.0) -
				t41 * t42 * 2.0) /
			2.0);
	leg->Torque_b =(c_a_tmp *
		(b_T_tmp * t94 * T_tmp_tmp * 4.0 - c_T_tmp * t97 * T_tmp_tmp * 4.0) /
		2.0 +
		t25 * ((T_tmp - l2 * (a * T_tmp_tmp) * 2.0) / b_a_tmp -
			t40 * (T_tmp + l2 * (t26 * T_tmp_tmp) * 2.0)));
}
//temp_leg->phi_b back temp_leg->phi_f front
void Vlink::leg_pos(struct legModel* leg) {
	double t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t23, t24, t25, 
		t26, t27, t28, t29, t30, t31, t32, t34, t35, t36, t37, t38, t39;//t33,t22
	t2 = cos(leg->phi_b->now());
	t3 = cos(leg->phi_f->now());
	t4 = sin(leg->phi_b->now());
	t5 = sin(leg->phi_f->now());
	t6 = pow(l2,2);
	t7 = pow(l3,2);
	t12 = l5 / 2.0;
	t8 = l1 * t2;
	t9 = l4 * t3;
	t10 = l1 * t4;
	t11 = l4 * t5;
	t13 = -t7;
	t16 = -t12;
	t14 = -t8;
	t15 = -t11;
	t17 = t10 + t15;
	t18 = l5 + t9 + t14;
	t19 = pow(t17, 2);
	t20 = pow(t18, 2);
	t21 = l2 * t17 * 2.0;
	t23 = l2 * t18 * 2.0;
	//t22 = -t21;
	t24 = t6 * t19 * 4.0;
	t25 = t6 * t20 * 4.0;
	t26 = t6 + t13 + t19 + t20;
	t27 = pow(t26, 2);
	t29 = t23 + t26;
	t28 = -t27;
	t30 = 1.0 / t29;
	t31 = t24 + t25 + t28;
	t32 = sqrt(t31);
	//t33 = t22 + t32;
	t34 = -t30 * (t21 - t32);
	t35 = atan(t34);
	t36 = t35 * 2.0;
	t37 = cos(t36);
	t38 = sin(t36);
	t39 = l2 * t37;
	leg->l0 = sqrt(pow((t8 + t16 + t39), 2) + pow((t10 + l2 * t38), 2));
	leg->phi0 = atan2(t10 + l2 * t38, t8 + t16 + t39);
	leg->state[0] = (leg->phi0 - 0.5 * pi - leg->state[4]);
	leg->state[3] = -leg->foot_angle->d * radius;
	leg->state[2] += (leg->last_dx + leg->state[3]) * timeStep / 2000;
	//leg->state[2] = 0;
	//leg->state[3] = 0;
	leg->last_dx = leg->state[3];
	leg->state[4] = angle[1];
	leg->state[5] = -dangle[1];
	//state space calc
	
}
void Vlink::leg_spd(struct legModel* leg) {
	double a_tmp;
	double b_a_tmp;
	double b_spd_tmp;
	double c_spd_tmp;
	double spd_tmp;
	double t10;
	double t101;
	double t105;
	double t11;
	double t23;
	double t24;
	double t25;
	double t26;
	double t40;
	double t41;
	double t42;
	double t48;
	double t49;
	double t56;
	double t6;
	double t66;
	double t67;
	double t74;
	double t8;
	double t80;
	double t82_tmp;
	double t9;
	double t94;
	double t97;
	//     This function was generated by the Symbolic Math Toolbox version 9.2.
	//     26-Jul-2023 17:07:38
	t6 = l2 * l2;
	t8 = l1 * std::cos(leg->phi_b->now());
	t9 = l4 * std::cos(leg->phi_f->now());
	t10 = l1 * std::sin(leg->phi_b->now());
	t11 = l4 * std::sin(leg->phi_f->now());
	t23 = t10 - t11;
	t24 = (l5 + t9) - t8;
	t25 = t23 * t23;
	t26 = t24 * t24;
	t40 = t8 * t23 * 2.0 + t10 * t24 * 2.0;
	t41 = t9 * t23 * 2.0 + t11 * t24 * 2.0;
	t42 = ((t6 - l3 * l3) + t25) + t26;
	t48 = 1.0 / (l2 * t24 * 2.0 + t42);
	t49 = t48 * t48;
	t25 = std::sqrt((t6 * t25 * 4.0 + t6 * t26 * 4.0) - t42 * t42);
	t56 = 1.0 / t25;
	a_tmp = l2 * t23 * 2.0 - t25;
	t25 = std::atan(-t48 * a_tmp) * 2.0;
	t66 = std::cos(t25);
	t67 = std::sin(t25);
	t74 = 1.0 / (t49 * (a_tmp * a_tmp) + 1.0);
	t25 = l2 * t66;
	t26 = l2 * t67;
	t80 = t10 + t26;
	t82_tmp = (t8 - l5 / 2.0) + t25;
	t94 = (t10 + l2 * 0.0) + t26;
	b_a_tmp = t82_tmp - l2 * 0.0;
	t97 = b_a_tmp * b_a_tmp;
	t101 = 1.0 / std::sqrt(t80 * t80 + t82_tmp * t82_tmp);
	t105 = 1.0 / (t94 * t94 + t97);
	spd_tmp =
		-(l2 * t10 * 2.0 + t40) * t49 * a_tmp +
		t48 * (l2 * t8 * 2.0 - t56 *
			((t6 * t8 * t23 * 8.0 + t6 * t10 * t24 * 8.0) -
				t40 * t42 * 2.0) /
			2.0);
	b_spd_tmp = t25 * t74;
	c_spd_tmp = t26 * t74;
	t40 =
		-(l2 * t11 * 2.0 + t41) * t49 * a_tmp +
		t48 * (l2 * t9 * 2.0 - t56 *
			((t6 * t9 * t23 * 8.0 + t6 * t11 * t24 * 8.0) -
				t41 * t42 * 2.0) /
			2.0);
	leg->dl0->update(leg->phi_b->d * t101 *
			(t80 * (t8 - b_spd_tmp * spd_tmp * 2.0) * 2.0 -
				t82_tmp * (t10 - c_spd_tmp * spd_tmp * 2.0) * 2.0) /
			2.0 +
			leg->phi_f->d * t101 *
			(b_spd_tmp * t80 * t40 * 4.0 - c_spd_tmp * t82_tmp * t40 * 4.0) /
			2.0);
		b_spd_tmp = l2 * 0.0 * 2.0;
		c_spd_tmp = -t66 * t74;
		t26 = -t67 * t74;
		t25 = t94 * (1.0 / t97);
		leg->dphi0 =
			leg->phi_b->d * t97 * t105 *
			(((t8 - b_spd_tmp) + l2 * (c_spd_tmp * spd_tmp) * 2.0) / b_a_tmp +
				t25 * ((t10 + b_spd_tmp) + l2 * (t26 * spd_tmp) * 2.0)) +
			leg->phi_f->d * t97 * t105 *
			((b_spd_tmp - l2 * (c_spd_tmp * t40) * 2.0) / b_a_tmp -
				t25 * (b_spd_tmp + l2 * (t26 * t40) * 2.0));
		leg->state[1] = (leg->dphi0 - leg->state[5]);
		leg->dtheta->update(leg->state[1]);
}
void Vlink::FN_solve(legModel* leg) 
{
	double P;
	double cos_theta;
	double sin_theta;
	double F;
	double Tp;
	double L0;
	double dL0;
	double ddL0;
	double theta;
	double dtheta;
	double ddtheta;
	double ddzw;
	double ddzm;
	theta = leg->state[0];
	dtheta = leg->state[1];
	ddtheta = leg->dtheta->d;
	F = leg->F;
	Tp = leg->Tp;
	L0 = leg->l0;
	dL0 = leg->dl0->now();
	ddL0 = leg->dl0->d;
	cos_theta = std::cos(theta);
	sin_theta = std::sin(theta);
	ddzm = ddx[2]*cos(angle[1])*cos(angle[0])-g;
	ddzw = ddzm - ddL0 * cos_theta + 2 * dL0 * dtheta * sin_theta + L0 * ddtheta * sin_theta + L0 * dtheta * dtheta * cos_theta;
	P = F * cos_theta + Tp * sin_theta / L0;
	leg->FN = P + m_wheel * g + m_wheel * ddzw;
	//cout << "ddzw" << ddzw << "  FN" << lleg->FN << endl;
}
