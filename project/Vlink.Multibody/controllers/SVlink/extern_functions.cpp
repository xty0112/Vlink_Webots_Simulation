#include "Vlink.h"
double lqr_index[12][4];
double pid_index[pid_num * 3];
bool flag = false;
void pid_file_read(void) {
    FILE* file;
    if ((file = fopen("pid_index.csv", "r")) == NULL) {
        cout << "Can't open pid_index!" << endl;
    }
    fseek(file, 0, SEEK_SET);
    for (size_t i = 0; i < pid_num * 3; i++)
    {
        fscanf(file, "%lf", &pid_index[i]);
        cout << "pid_index[i]=" << pid_index[i] << endl;
        fseek(file, 1L, SEEK_CUR);
    }
}
void lqr_file_read(void) {
    FILE* file;
    if ((file = fopen("lqr_index.csv", "r")) == NULL) {
        cout << "Can't open lqr_index!" << endl;
    }
    for (size_t i = 0; i < 12; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            fscanf(file, "%lf", &lqr_index[i][j]);
            fseek(file, 1L, SEEK_CUR);
            cout << lqr_index[i][j] << "  ";
        }
        cout << endl;
        cout << endl;
    }
}
void Vlink::file_print(double a1, double a2, double a3, double a4, double a5, double a6) {
    if (!flag)
    {
        ofile.open("test.xls", ios::out | ios::trunc);
        ofile << "控制周期" << "\t" << "属性1" << "\t" << "属性2" << "\t" << "属性3" << "\t" << "属性4" << "\t" << "属性5" << "\t" << "属性6"
            << "\t" << "说明控制状态：前进=1 后退=2 左转=3 右转=4 升高=5 降低=6 闲置=0" << endl;
        flag = true;
    }
    //属性1        属性2          属性3         属性4          属性5
    ofile << count << "\t" << control_state << "\t" << a1 << "\t" << a2 << "\t" << a3 << "\t" << a4 << "\t" << a5 << "\t" << a6 << endl;
}
