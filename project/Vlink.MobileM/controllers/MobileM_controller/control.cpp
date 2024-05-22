#include "Vlink.h"
Path *path_generate(void);
void Vlink::control(void)
{
    lleg->exp_state[2]+=(double)timeStep/1000*lleg->exp_state[3];
    rleg->exp_state[2]+=(double)timeStep/1000*rleg->exp_state[3];
    // �ȳ�pid���� ��������������ƺ��ȳ�����
    leg_length_control();
    // �������㼰���Ĳ���
    COM_calc();
    COM_comp();
    // �����ȳ�ӳ��lqr��������
    lqr_k1(lleg);
    lqr_k1(rleg);
    // ����֧�����ж����״̬
    FN_solve(lleg);
    FN_solve(rleg);
    // ����lqr����
    lqr_fdb(lleg);
    lqr_fdb(rleg);
    // ���ȽǶ�һ�²���
    theta_pid->DTS_update(lleg->state[0] - rleg->state[0]);
    lleg->Tp -= theta_pid->control_calc();
    rleg->Tp += theta_pid->control_calc();
    // ת�����
    turn_control();
    // ����ģ�Ϳ�����ת��ʵ�ʿ�����
    leg_conv(lleg);
    leg_conv(rleg);

    inverse_kine(Manipulator_L);
    inverse_kine(Manipulator_R);
    dynamics(Manipulator_L);
    dynamics(Manipulator_R);
    cout<<"there    "<<height_exp<<endl;
}
void Vlink::keyboard_control_exp_set(void)
{
    int k = key->getKey();
    driver_flag = true;
    if (driver_flag)
    {
        double compx = 0;
        switch (k)
        {

        case 'W': // key ��
            control_state = 1;
            lleg->exp_state[3] = 0.2;
            rleg->exp_state[3] = 0.2;
            cout << "move forward" << endl;
            break;
        case 'S': // key ��
            control_state = 2;
            lleg->exp_state[3] = -0.2;
            rleg->exp_state[3] = -0.2;
            cout << "move backward" << endl;
            break;
        case 'A': // key ��
            control_state = 3;
            exp_angle[2] += 0.02;
            lleg->exp_state[2] -= 0.02 * pi * half_wheel_d;
            rleg->exp_state[2] += 0.02 * pi * half_wheel_d;
            cout << "turn left" << endl;
            break;
        case 'D': // key ��
            control_state = 4;
            exp_angle[2] -= 0.02;
            lleg->exp_state[2] += 0.02 * pi * half_wheel_d;
            rleg->exp_state[2] -= 0.02 * pi * half_wheel_d;
            cout << "turn_right" << endl;
            break;

        case 'I':
            control_state = 5;
            height_exp += 0.001;
            break;
        case 'K':
            control_state = 6;
            height_exp -= 0.001;
            break;
        case 'G':
            // Manipulator_L->exp_set(true, -0.1, 0, 0);
            // Manipulator_R->exp_set(true, 0.1, 0, 0);
            break;
        case 'Y':
            // Manipulator_L->exp_set(true, 0, 0.1, 0);
            // Manipulator_R->exp_set(true, 0, -0.1, 0);
            break;
        case 'H':
            // Manipulator_L->exp_set(true, 0, -0.1, 0);
            // Manipulator_R->exp_set(true, 0, 0.1, 0);
            break;
        case 'U':
            // Manipulator_L->exp_set(true, 0, 0, 0.1);
            // Manipulator_R->exp_set(true, 0, 0, -0.1);
            break;
        case 'J':
            // Manipulator_L->exp_set(true, 0, 0, -0.1);
            // Manipulator_R->exp_set(true, 0, 0, 0.1);
            break;
        case 'X':
            // Manipulator_L->exp_set(false, 0, 0, 0);
            // Manipulator_R->exp_set(false, 0, 0, 0);
            break;
        case 'C':
            mode = CONTROL;
            break;
        case 'V':
            break;
        case 'P':
            mode = PATH;
            followtest->set_path(path_generate());
        case '0':
            m_control_mode = true;
        case '1':
            m_control_mode = false;
            x = -0.107335;
            y = 0;
            z = -0.005084;
            break;
        case '+':
            y -= 0.001;
            break;
        case '2':
            x = -0.102;
            z = 0.051;
            break;
        case '-':
            y += 0.001;
            break;
        default:
            control_state = 0;
            lleg->exp_state[3] = 0;
            rleg->exp_state[3] = 0;
            break;
        }
    }
    if (exp_angle[2] > pi && angle[2] < 0)
    {
        exp_angle[2] -= 2 * pi;
    }
    if (exp_angle[2] < -pi && angle[2] > 0)
    {
        exp_angle[2] += 2 * pi;
    }
    turn_pid->exp_set(exp_angle[2]);
    l0_pid_l->exp_set(height_exp);
    l0_pid_r->exp_set(height_exp);
    Manipulator_L->exp_pos[0] = x;
    Manipulator_L->exp_pos[1] = y;
    Manipulator_L->exp_pos[2] = z;
    Manipulator_R->exp_pos[0] = x;
    Manipulator_R->exp_pos[1] = y;
    Manipulator_R->exp_pos[2] = z;
    cout << "x=" << x << " y=" << y << " z=" << z << endl;
}
void Vlink::path_follow_exp_set(PathFollow *pathfollow)
{
    if (abs(pathfollow->angle_out()) > 0.5)
    {
        exp_angle[2] = angle[2] + 0.75 * pathfollow->angle_out();
    }
    else
    {
        exp_angle[2] = angle[2] + pathfollow->angle_out();
    }
    turn_pid->exp_set(exp_angle[2]);
    lleg->exp_state[3] = pathfollow->exp_out();
    rleg->exp_state[3] = pathfollow->exp_out();
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
    theta_compensator = -atan(COM_proj / height_exp) / 4;
    cout << "theta_comp=" << theta_compensator << endl;
    if (theta_compensator > -1 && theta_compensator < 0)
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

double hip_maxT(double input)
{
    if (input > hip_max_T)
    {
        cout << AnsiCodes::YELLOW_FOREGROUND << "reached max_Torque" << AnsiCodes::RESET << endl;
        return hip_max_T;
    }
    else
    {
        if (input < -hip_max_T)
        {
            cout << AnsiCodes::YELLOW_FOREGROUND << "reached negative_max_Torque" << AnsiCodes::RESET << endl;
            return -hip_max_T;
        }
        else
        {
            return input;
        }
    }
}
double wheel_maxT(double input)
{
    if (input > wheel_max_T)
    {
        cout << AnsiCodes::YELLOW_FOREGROUND << "reached max_Torque" << AnsiCodes::RESET << endl;
        return wheel_max_T;
    }
    else
    {
        if (input < -wheel_max_T)
        {
            cout << AnsiCodes::YELLOW_FOREGROUND << "reached negative_max_Torque" << AnsiCodes::RESET << endl;
            return -wheel_max_T;
        }
        else
        {
            return input;
        }
    }
}
void Vlink::apply_control(legModel *leg)
{

    leg->hip_f->setTorque(hip_maxT(leg->Torque_f));
    leg->hip_b->setTorque(hip_maxT(leg->Torque_b));
    leg->wheel->setTorque(wheel_maxT(leg->T_foot));
}
