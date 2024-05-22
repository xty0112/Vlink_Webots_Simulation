#include "Vlink.h"
Path* path_generate(void);
void Vlink::control(void)
{
    //�ȳ�pid���� ��������������ƺ��ȳ�����
    leg_length_control();
    //�������㼰���Ĳ���
    COM_calc();
    COM_comp();
    //�����ȳ�ӳ��lqr��������
    lqr_k1(lleg);
    lqr_k1(rleg);
    //����֧�����ж����״̬
    FN_solve(lleg);
    FN_solve(rleg);
    //����lqr����
    lqr_fdb(lleg);
    lqr_fdb(rleg);
    //���ȽǶ�һ�²���
    theta_pid->DTS_update(lleg->state[0] - rleg->state[0]);
    lleg->Tp -= theta_pid->control_calc();
    rleg->Tp += theta_pid->control_calc();
    //ת�����
    turn_control();
    //����ģ�Ϳ�����ת��ʵ�ʿ�����
    leg_conv(lleg);
    leg_conv(rleg);
}
void Vlink::keyboard_control_exp_set(void) {
    int k = key->getKey();
    if (receiver->getQueueLength() > 0) {
        string message((const char*)receiver->getData());
        receiver->nextPacket();
        if (message.compare("1")==0)
        {
            driver_flag = true;
        }
        if (message.compare("0")==0)
        {
            driver_flag = false;
        }
    }
    if (driver_flag)
    {
        switch (k)
        {
        case 'W'://key ��
            control_state = 1;
            lleg->exp_state[3] = 1;
            rleg->exp_state[3] = 1;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
            cout << "move forward" << endl;
            break;
        case 'S'://key ��
            control_state = 2;
            lleg->exp_state[3] = -1;
            rleg->exp_state[3] = -1;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
            cout << "move backward" << endl;
            break;
        case 'A'://key ��
            control_state = 3;
            exp_angle[2] += 0.03;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
            cout << "turn left" << endl;
            break;
        case 'D'://key ��
            control_state = 4;
            exp_angle[2] -= 0.03;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
            cout << "turn_right" << endl;
            break;
        case'I':
            control_state = 5;
            height_exp += 0.001;
            break;
        case'K':
            control_state = 6;
            height_exp -= 0.001;
            break;
        case 'T':
            Manipulator_L->exp_set(true, 0.1, 0, 0);
            Manipulator_R->exp_set(true, -0.1, 0, 0);
            break;
        case 'G':
            Manipulator_L->exp_set(true, -0.1, 0, 0);
            Manipulator_R->exp_set(true, 0.1, 0, 0);
            break;
        case 'Y':
            Manipulator_L->exp_set(true, 0, 0.1, 0);
            Manipulator_R->exp_set(true, 0, -0.1, 0);
            break;
        case 'H':
            Manipulator_L->exp_set(true, 0, -0.1, 0);
            Manipulator_R->exp_set(true, 0, 0.1, 0);
            break;
        case 'U':
            Manipulator_L->exp_set(true, 0, 0, 0.1);
            Manipulator_R->exp_set(true, 0, 0, -0.1);
            break;
        case 'J':
            Manipulator_L->exp_set(true, 0, 0, -0.1);
            Manipulator_R->exp_set(true, 0, 0, 0.1);
            break;
        case 'X':
            Manipulator_L->exp_set(false, 0, 0, 0);
            Manipulator_R->exp_set(false, 0, 0, 0);
            break;
        case 'C':
            magnet_on = true;
            break;
        case 'V':
            magnet_on = false;
            magnet_l->unlock();
            magnet_r->unlock();
            break;
        case 'P':
            mode = PATH;
            followtest->set_path(path_generate());
        default:
            control_state = 0;
            lleg->exp_state[3] = 0;
            rleg->exp_state[3] = 0;
            break;
        }
    }
    if (exp_angle[2]>pi&&angle[2]<0)
    {
        exp_angle[2] -= 2 * pi;
    }
    if (exp_angle[2]<-pi && angle[2] > 0)
    {
        exp_angle[2] += 2 * pi;
    }
    if (magnet_on)
    {
        magnet_l->lock();
        magnet_r->lock();
    }
    if (!magnet_on)
    {
        magnet_l->unlock();
        magnet_r->unlock();
    }
    turn_pid->exp_set(exp_angle[2]);
    l0_pid_l->exp_set(height_exp);
    l0_pid_r->exp_set(height_exp);
}
void Vlink::path_follow_exp_set(PathFollow* pathfollow)
{
    if (abs(pathfollow->angle_out())>0.5)
    {
        exp_angle[2] = angle[2] + 0.75*pathfollow->angle_out();
    }
    else
    {
        exp_angle[2] = angle[2] + pathfollow->angle_out();
    }
    turn_pid->exp_set(exp_angle[2]);
    lleg->exp_state[3] = pathfollow->exp_out();
    rleg->exp_state[3] = pathfollow->exp_out();
    lleg->exp_state[2] = 0;
    rleg->exp_state[2] = 0;
    lleg->state[2] = 0;
    rleg->state[2] = 0;
}
void Vlink::turn_control(void)
{
    turn_pid->DTS_update(angle[2]);
    lleg->T_foot += turn_pid->control_calc();
    rleg->T_foot -= turn_pid->control_calc();
}
void Vlink::leg_length_control(void)
{
    roll_pid->DTS_update(angle[0]);
    double leg_fix = roll_pid->control_calc();
    l0_pid_l->DTS_update(lleg->l0);
    l0_pid_r->DTS_update(rleg->l0);
    lleg->F = l0_pid_l->control_calc() + leg_fix;
    rleg->F = l0_pid_r->control_calc() - leg_fix;
}
void Vlink::COM_comp(void) 
{
    double theta_compensator;
    theta_compensator = -atan(COM_proj / height_exp)/4;
    //cout <<"theta_comp=" << theta_compensator << endl;
    if (theta_compensator>-1&& theta_compensator < 0)
    {
        lleg->exp_state[0] = theta_compensator;
        rleg->exp_state[0] = theta_compensator;
    }
    else
    {
        lleg->exp_state[0] = -0.3;
        rleg->exp_state[0] = -0.3;
    }
}

double motor_maxT(double input) {
    if (input > max_T)
    {
        cout << AnsiCodes::YELLOW_FOREGROUND << "reached max_Torque" << AnsiCodes::RESET << endl;
        return max_T;
    }
    else
    {
        if (input < -max_T)
        {
            cout << AnsiCodes::YELLOW_FOREGROUND << "reached negative_max_Torque" << AnsiCodes::RESET << endl;
            return -max_T;
        }
        else
        {
            return input;
        }
    }
}
void Vlink::apply_control(legModel* leg, bool mode) {
    if (mode == true)
    {
        leg->hip_f->setTorque(motor_maxT(leg->Torque_f));
        leg->hip_b->setTorque(motor_maxT(leg->Torque_b));
    }
    leg->wheel->setTorque(motor_maxT(leg->T_foot));
}



