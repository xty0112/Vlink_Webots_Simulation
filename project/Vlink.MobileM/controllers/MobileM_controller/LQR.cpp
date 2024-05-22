#include "Vlink.h"
void Vlink::lqr_k1(legModel* leg) {
    double t1, t2, t3;
    t1 = leg->l0;
    t2 = pow(t1, 2);
    t3 = pow(t1, 3);
    leg->lqr_line1[0] = t1 * lqr_index[0][2] + t2 * lqr_index[0][1] + t3 * lqr_index[0][0] + lqr_index[0][3];
    leg->lqr_line2[0] = t1 * lqr_index[1][2] + t2 * lqr_index[1][1] + t3 * lqr_index[1][0] + lqr_index[1][3];
    leg->lqr_line1[1] = t1 * lqr_index[2][2] + t2 * lqr_index[2][1] + t3 * lqr_index[2][0] + lqr_index[2][3];
    leg->lqr_line2[1] = t1 * lqr_index[3][2] + t2 * lqr_index[3][1] + t3 * lqr_index[3][0] + lqr_index[3][3];
    leg->lqr_line1[2] = t1 * lqr_index[4][2] + t2 * lqr_index[4][1] + t3 * lqr_index[4][0] + lqr_index[4][3];
    leg->lqr_line2[2] = t1 * lqr_index[5][2] + t2 * lqr_index[5][1] + t3 * lqr_index[5][0] + lqr_index[5][3];
    leg->lqr_line1[3] = t1 * lqr_index[6][2] + t2 * lqr_index[6][1] + t3 * lqr_index[6][0] + lqr_index[6][3];
    leg->lqr_line2[3] = t1 * lqr_index[7][2] + t2 * lqr_index[7][1] + t3 * lqr_index[7][0] + lqr_index[7][3];
    leg->lqr_line1[4] = t1 * lqr_index[8][2] + t2 * lqr_index[8][1] + t3 * lqr_index[8][0] + lqr_index[8][3];
    leg->lqr_line2[4] = t1 * lqr_index[9][2] + t2 * lqr_index[9][1] + t3 * lqr_index[9][0] + lqr_index[9][3];
    leg->lqr_line1[5] = t1 * lqr_index[10][2] + t2 * lqr_index[10][1] + t3 * lqr_index[10][0] + lqr_index[10][3];
    leg->lqr_line2[5] = t1 * lqr_index[11][2] + t2 * lqr_index[11][1] + t3 * lqr_index[11][0] + lqr_index[11][3];
}
double matrix_multipul(double* state, double* power, bool mode)
{
    double temp = 0;
    int multipul_num;
    if (mode == true) multipul_num = 6;
    else multipul_num = 2;
    for (int i = 0; i < multipul_num; i++)
    {
        temp += *power * *state;
        power++;
        state++;
    }
    return  temp;
}
void Vlink::lqr_fdb(struct legModel* leg) {
    double temp1;
    double temp2;
    bool isRobotOnGround;
    //if (leg->FN > 10)
    if (true)
    {
        isRobotOnGround = true;
    }
    else
    {
        isRobotOnGround = false;
        cout << AnsiCodes::YELLOW_FOREGROUND << "robot offground" << AnsiCodes::RESET << endl;
    }
    temp1 = matrix_multipul(leg->state, leg->lqr_line1, true) - matrix_multipul(leg->exp_state, leg->lqr_line1, true);
    temp2 = matrix_multipul(leg->state, leg->lqr_line2, isRobotOnGround) - matrix_multipul(leg->exp_state, leg->lqr_line2, isRobotOnGround);
    if (isRobotOnGround) { leg->T_foot = temp1; }
    else { leg->T_foot = 0; }
    leg->Tp = temp2;
}