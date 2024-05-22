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
        ofile << "��������" << "\t" << "����1" << "\t" << "����2" << "\t" << "����3" << "\t" << "����4" << "\t" << "����5" << "\t" << "����6"
            << "\t" << "˵������״̬��ǰ��=1 ����=2 ��ת=3 ��ת=4 ����=5 ����=6 ����=0" << endl;
        flag = true;
    }
    //����1        ����2          ����3         ����4          ����5
    ofile << count << "\t" << control_state << "\t" << a1 << "\t" << a2 << "\t" << a3 << "\t" << a4 << "\t" << a5 << "\t" << a6 << endl;
}
