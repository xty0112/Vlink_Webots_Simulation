// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications
#include "Vlink.h"
// VMC���������phi����ʵ��alpha IMU����������phi
/*
 */
void pid_file_read(void);
void lqr_file_read(void);
int main(int argc, char **argv)
{
    pid_file_read();
    lqr_file_read();
    Vlink *vlink = new Vlink();
    vlink->run();
    cout << "simulation_end" << endl;
    delete vlink;
    return 0;
}

void Vlink::run()
{
    while (step(timeStep) != -1)
    {
        // ��ȡ��������Ϣ�����¸��۲� ���ı�״̬����ֵ
        observe();
        virtual_model_print();
        Torque_print();
        // ʩ�������ź�
        switch (mode)
        {
        case Vlink::CONTROL:
            keyboard_control_exp_set();
            control();
            if (magnet_l->isLocked() == true || magnet_r->isLocked() == true)
            {
                mode = CONNECT;
            }
            // �����̨����excel�ĵ�����������
            console_print();
            if (control_flag)
            {
                // ��е���ƶ�
                Manipulator_L->move_exp();
                Manipulator_R->move_exp();
                // �Źؽ� �ֵ��ʩ�����ؿ���
                apply_control(lleg, !switch_flag);
                apply_control(rleg, !switch_flag);
            }
            count++;
            if (count > 1)
            {
                control_flag = true;
            }
            break;
        case Vlink::PATH:
            path_follow_exp_set(followtest);
            control();
            break;
        case Vlink::CONNECT:
            keyboard_control_exp_set();
            control();
            console_print();
            if (control_flag)
            {
                apply_control(lleg, !switch_flag);
                apply_control(rleg, !switch_flag);
            }
            break;
        default:
            break;
        }
    }
}
/*
ת��ʱ�����棬��ת�Ƕȣ����ȽǶ�һ�¿��Ƶ�pid�����������ӻ��ֻ��ڣ����Ƕ�ת����Դ󣬲���������pid
ˮƽת��ת����Ȧ��ͻȻ���£������Ƕȴ���pi ��С��-pi����Ҫȡģ

�������Ȳ��Գƣ������Ǹ��Ƶ�ʱ���������˸ģ����Դ��������ȹ�ʽ������һ�����õ�����һ���ȵ�
��ͷˮƽ��theta��� ��ǰ���£�
*/
