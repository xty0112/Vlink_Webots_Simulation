#include "Vlink.h"

void Vlink::console_print(void)
{
    cout << endl << count << endl;
    Torque_print();
    //state_print(false , true, false);
    virtual_model_print();
    //leg_print(rleg);
    //leg_print(lleg);
    //manipulator_print(Manipulator_L);
    //COM_print();
    //pid_print(turn_pid);
    //state_print(false, true, false);
    //lqr_print();
    //imu_print();
    //GPS_print();
    //file_print(lleg->Torque_b,lleg->Torque_f,lleg->T_foot,state_l->dx,state_l->phi);
    //file_print(state_l->test[0], state_l->test[1], state_l->test[2], state_l->test[3], state_l->test[4], state_l->test[5]);
    //file_print(gps_pos[0]->now(), gps_pos[2]->now(), path1(gps_pos[0]->now()), 0, 0, 0);
    //file_print(gps_pos[0]->now(),gps_pos[2]->now(),0,0,0,0);
    //
}
void Vlink::Torque_print() {
    cout << "Torque_lf=" << lleg->Torque_f << ",Torque_rf=" << rleg->Torque_f << endl;
    cout << "Torque_lb=" << lleg->Torque_b << ",Torque_rb=" << rleg->Torque_b << endl;
    cout << "T_Foot_l=" << lleg->T_foot << ",T_Foot_r=" << rleg->T_foot << endl;
}
void Vlink::state_print(bool f1, bool f2, bool f3) {
    /*
    cout << "right:" << endl;
    if (f1)
    {
        cout << "theta_r=" << state_r->theta << ",dtheta_r=" << state_r->dtheta << endl;
    }
    if (f2)
    {
        cout << "x=" << state_r->x << ",dx=" << state_r->dx << endl;
    }
    if (f3)
    {
        cout << "phi=" << state_r->phi << ",dphi=" << state_r->dphi << endl;
    }
    cout << "left:" << endl;
    if (f1)
    {
        cout << "theta_l=" << state_l->theta << ",dtheta_l=" << state_l->dtheta << endl;
    }
    if (f2)
    {
        cout << "x=" << state_l->x << ",dx=" << state_l->dx << endl;
    }
    if (f3)
    {
        cout << "phi=" << state_l->phi << ",dphi=" << state_l->dphi << endl;
    }
    */
}
void Vlink::virtual_model_print() {
    cout << "F_l=" << lleg->F << ",Tp_l=" << lleg->Tp << endl;
    cout << "F_r=" << rleg->F << ",Tp_r=" << rleg->Tp << endl;
}
void Vlink::leg_print(legModel* leg) {
    if (leg->flag==right_leg)
    {
        cout << "right_leg_print" << endl;
    }
    else
    {
        cout << "left_leg_print" << endl;
    }
    cout << "l0=" << leg->l0 << ",dl0=" << leg->dl0 << endl;
    cout << "phi0=" << leg->phi0 << ",dphi0=" << leg->dphi0 << endl;
    cout << "phi_b=" << leg->phi_b->now() << ",phi_f=" << leg->phi_f->now() << endl;
}
void PID::pid_print()
{
    cout << "exp=" << exp << endl;
    cout << "DTS=" << DTS[0]<<"   "<<DTS[1]<<"   "<<DTS[2] << endl;
    cout << "control=" << kp * (DTS[0] - DTS[1])<<"(kp*)+"
        << ki * DTS[0]<<"(ki*)+"
        <<kd * (DTS[0] - 2 * DTS[1] + DTS[2])<<"(kd*)="<<control_calc()<< endl;
}
void Vlink::pid_print(PID* pid) {
    pid->pid_print();
}
void Vlink::manipulator_print(manipulator* M) {
    M->position_print();
}
void Vlink::odo_print() 
{
    cout << "odo_print" <<endl ;
    cout << "left_leg,speed=" << lleg->foot_angle->d << ",angle=" << lleg->foot_angle->now() << endl;
    cout << "right_leg,speed=" << rleg->foot_angle->d << ",angle=" << rleg->foot_angle->now() << endl;
}
void Vlink::imu_print()
{
    cout << "imu_print" << endl;
    cout << "roll=" << angle[0] << ",pitch=" << angle[1] << ",yaw=" << angle[2] << endl;
    cout << "droll=" << dangle[0] << ",dpitch=" << dangle[1] << ",dyaw=" << dangle[2] << endl;

}
void Vlink::COM_print()
{
    cout << "d(the center of mass) is" << COM->d << endl;
}
void Vlink::lqr_print() {
/*
    cout << "left_lqr" << endl;
    cout << "line1=" << lqr_line1_l[0] << "  " << lqr_line1_l[1] << " " << lqr_line1_l[2] << " " << lqr_line1_l[3] << " " << lqr_line1_l[4] << " " << lqr_line1_l[5] << endl;
    cout << "line2=" << lqr_line2_l[0] << "  " << lqr_line2_l[1] << " " << lqr_line2_l[2] << " " << lqr_line2_l[3] << " " << lqr_line2_l[4] << " " << lqr_line2_l[5] << endl;
    cout << "right_lqr" << endl;
    cout << "line1=" << lqr_line1_r[0] << "  " << lqr_line1_r[1] << " " << lqr_line1_r[2] << " " << lqr_line1_r[3] << " " << lqr_line1_r[4] << " " << lqr_line1_r[5] << endl;
    cout << "line2=" << lqr_line2_r[0] << "  " << lqr_line2_r[1] << " " << lqr_line2_r[2] << " " << lqr_line2_r[3] << " " << lqr_line2_r[4] << " " << lqr_line2_r[5] << endl;
*/
}
void Vlink::GPS_print()
{
    cout << "GPS_print" << endl;
    cout << "x=" << gps_pos[0]->now() << ",y=" << gps_pos[1]->now() << ",z=" << gps_pos[2]->now() << endl;
    cout << "dx=" << gps_pos[0]->d << ",dy=" << gps_pos[1]->d << ",dz=" << gps_pos[2]->d << endl;

}
