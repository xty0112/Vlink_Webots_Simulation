// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications
#include "Vlink.h"
//VMC���������phi����ʵ��alpha IMU����������phi
/*
*/
void pid_file_read(void);
void lqr_file_read(void);
int main(int argc, char** argv) {
    lqr_file_read();
    pid_file_read();
    Vlink* vlink = new Vlink();
    vlink->run();
    cout << "simulation_end" << endl;
    delete vlink;
    return 0;
}

void Vlink::run() {
    while (step(timeStep) != -1) {
        //��ȡ��������Ϣ�����¸��۲� ���ı�״̬����ֵ
        observe();
        //ʩ�������ź�
        switch (mode)
        {
        case Vlink::CONTROL:
            keyboard_control_exp_set();
            control();
            if (control_flag)
            {
                //��е���ƶ�
                angle_control(Manipulator_L);
                angle_control(Manipulator_R);
                //�Źؽ� �ֵ��ʩ�����ؿ���
                apply_control(lleg);
                apply_control(rleg);
            }
            cout<<"                  "<<lleg->state[2]<<endl;
            console_print();
            break;
        case Vlink::PATH:
            path_follow_exp_set(followtest);
            control();
            if (control_flag)
            {
                //��е���ƶ�
                //Manipulator_L->move_exp();
                //Manipulator_R->move_exp();
                //�Źؽ� �ֵ��ʩ�����ؿ���
                apply_control(lleg);
                apply_control(rleg);
            }
            break;
        case Vlink::STAND:
            keyboard_control_exp_set();
            control();
            break;
        default:
            break;
        }
        //�����̨����excel�ĵ�����������
        count++;
        if (count>1)
        {
            control_flag = true;
        }
    }
}
/*
ת��ʱ�����棬��ת�Ƕȣ����ȽǶ�һ�¿��Ƶ�pid�����������ӻ��ֻ��ڣ����Ƕ�ת����Դ󣬲���������pid
ˮƽת��ת����Ȧ��ͻȻ���£������Ƕȴ���pi ��С��-pi����Ҫȡģ

�������Ȳ��Գƣ������Ǹ��Ƶ�ʱ���������˸ģ����Դ��������ȹ�ʽ������һ�����õ�����һ���ȵ�
��ͷˮƽ��theta��� ��ǰ���£�
*/
