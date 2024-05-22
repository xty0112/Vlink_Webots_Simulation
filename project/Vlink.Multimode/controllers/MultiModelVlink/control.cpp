#include "Vlink.h"
Path *path_generate(void);
void Vlink::control(void)
{
    // 腿长pid控制 包括机身横滚控制和腿长设置
    leg_length_control();
    // 重心运算及重心补偿
    // COM_calc();
    // COM_comp();
    // 根据腿长映射lqr反馈矩阵
    lqr_k1(lleg);
    lqr_k1(rleg);
    // 解算支持力判断离地状态
    FN_solve(lleg);
    FN_solve(rleg);
    // 计算lqr反馈
    lqr_fdb(lleg);
    lqr_fdb(rleg);
    // 两腿角度一致补偿
    theta_pid->DTS_update(lleg->state[0] - rleg->state[0]);
    lleg->Tp -= theta_pid->control_calc();
    rleg->Tp += theta_pid->control_calc();
    // 转向控制
    turn_control();
    // 虚拟模型控制量转换实际控制量
    leg_conv(lleg);
    leg_conv(rleg);
}
void Vlink::keyboard_control_exp_set(void)
{
    int k = key->getKey();
    driver_flag = true;
    if (driver_flag)
    {

        switch (k)
        {
        case 'W': // key ↑
            control_state = 1;
            lleg->exp_state[3] = 1;
            rleg->exp_state[3] = 1;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
            cout << "move forward" << endl;
            break;
        case 'S': // key ↓
            control_state = 2;
            lleg->exp_state[3] = -1;
            rleg->exp_state[3] = -1;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
            cout << "move backward" << endl;
            break;
        case 'A': // key ←
            control_state = 3;
            exp_angle[2] += 0.03;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
            cout << "turn left" << endl;
            break;
        case 'D': // key →
            control_state = 4;
            exp_angle[2] -= 0.03;
            lleg->state[2] = 0;
            rleg->state[2] = 0;
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
        case 'R':
            mode=CONTROL;
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
    theta_compensator = -atan(COM_proj / height_exp) / 4;
    // cout <<"theta_comp=" << theta_compensator << endl;
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
