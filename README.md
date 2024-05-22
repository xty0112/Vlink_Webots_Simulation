# 五连杆轮足机器人仿真
## 简介
本工程基于webots平台实现了五连杆轮足机器人基础构型及几种改型的仿真，使用者可以利用本平台设计轮足机器人的应用场景，开展控制器测试、验证导航或规划算法等。
## 软件依赖
webots2023a
MATLAB2022b
Solidworks2020
## 目录结构
**本工程包含四个独立的小工程，使用者可以按照需求单独使用**
**在每个独立工程内，目录结构与webots wizard创建的工程结构相同** <br />
├── Readme.md                   // help  <br />
├── project                         // 项目工程 <br />
│   ├── Vlink.MultiMode             // 多模态五连杆轮足仿真 <br />
│   ├── Vlink.Manipulator         // 机械臂五连杆轮足仿真 <br />
│   ├── Vlink.MultiBody         // 五连杆轮足多机器人协同仿真 <br />
│   └── Manipulator              // 机械臂仿真 <br />
├── data                         // 仿真中读取的数据 <br />
└── model                         // 仿真SW模型 <br />
├── controllers//控制器 <br />
│   ├── **lqr_calc.m //lqr反馈矩阵计算器** <br />
│   ├── lqr_index.csv //lqr反馈矩阵数据 <br />
│   ├── pid_index.csv //pid参数数据 <br />
│   ├── test.xls //仿真读取的数据 <br />
│   ├── **Makefile //编译使用** <br />
│   ├── .cpp //源文件 <br />
│   └──  .h //头文件 <br />
├── libraries <br />
├── plugins <br />
├── protos <br />
│   ├── meshes//模型stl <br />
│   ├── urdf//机器人描述 <br />
│   │   ├── model.proto//proto描述 <br />
│   │   └── model.urdf//urdf描述 <br />
│   └── ...... <br />
└── worlds//wbt文件 <br />
## 使用方法
本节使用[Vlink.MultiMode](\Vlink.Multimode)工程为例
### &#x1F60A;如果您只是想看看
点击 Vlink.MultiMode文件夹->world文件夹->.wbt文件 即可直接打开仿真文件运行仿真，或点击[这里](\Vlink.Multimode\worlds\multimodel_Vlink.wbt)直接跳转。webots仿真中支持使用键盘对机器人进行控制，如果您想要控制机器人，参照
### &#x1F60A;如果您想要调节机器人控制器参数
如果您想要详细了解轮足机器人的控制算法，可以参考知乎韭菜的菜[这篇文章](https://zhuanlan.zhihu.com/p/563048952)。本工程的符号表示与控制方法与这篇文章中的定义大致相同，采用 **线性二次型调节器（LQR）** 控制轮足机器人的状态，并叠加了若干**PID控制器**来实现更复杂的控制效果。机器人状态可以用一个六维列向量表示，可以简单理解为机器人的腿部角度、位移、机身角度及他们的导数。
```math
x = \begin{bmatrix} 
\theta\\
\dot \theta\\
x\\
\dot x\\
\phi\\
\dot \phi\end{bmatrix}
u = \begin{bmatrix} 
T\\
Tp \end{bmatrix}
```
#### lqr参数调节
系统的控制器可以表述为`$\dot x=Ax+Bu$`，LQR就是通过最优控制的方法设计控制矩阵B。工程提供了一个lqr控制矩阵计算器，可以使用MATLAB2022b打开
点击 Vlink.MultiMode文件夹->controller文件夹->lqr\_calc.m，或点击[这里](\Vlink.Multimode\controllers\MultiModelVlink\lqr_calc.m)直接跳转。
```matlab
%权重矩阵参数
Q=diag([0.001 0.001 0.8 0.125 3 0.008]);
R=diag([1 0.01]);
```
在计算器内您可以找到这两个矩阵的赋值，更改这些值可以改变LQR的控制效果。Q矩阵的参数分别**独立的影响相对应状态的收敛速度，权重越大收敛越快**，并且这个权重似乎是与控制周期绝对相关的，即控制周期越小，这个参数也应越小。R矩阵的权重代表了**执行器输出的代价，权重越高代表代价越高**，调节器会倾向于更少使用这个执行器。此处轮电机的代价为1，髋关节电机的代价为0.01。
更改参数后，运行此MATLAB程序，不需要编译webots控制器，直接重新运行仿真即可，这是通过读写lqr_index.csv实现的。
#### pid参数调节
点击 Vlink.MultiMode文件夹->controller文件夹->pid\_index.csv，或点击[这里](\Vlink.Multimode\controllers\MultiModelVlink\pid_index.csv)直接跳转。打开后直接更改里面的参数即可，里面分别是腿长pid、两腿`$\theta$`角一致pid、转向pid、横滚角pid、~~位移pid~~，改完后记得ctrl+S。同样不需要编译webots控制器。
### &#x1F60A;如果您想要编辑控制器
本工程使用C++程序，Webots(gcc/Makefile)IDE，但由于代码量大，使用了多个源文件便于管理，推荐您使用VSCode打开controller文件夹进行编辑。如果您修改了控制器程序，**使用Webots文档编辑器打开controller文件夹中的[Makefile](\Vlink.Multimode\controllers\MultiModelVlink\Makefile)并编译**即可。如果您想要添加源文件，需要
```C++
#include “Vlink.h”
```
并在Makefile的C\++ Sources处添加上您的文件名
```makefile
### ---- C++ Sources ----
### if your program uses several C++ source files:
CXX_SOURCES = 
pathfollow.cpp 
timeFunc.cpp
test_functions.cpp 
control.cpp 
observe.cpp 
PID.cpp 
LQR.cpp 
extern_functions.cpp 
initialize.cpp 
main.cpp 
###
```
控制器通过Vlink类(定义在[Vlink.h](\Vlink.Multimode\controllers\MultiModelVlink\Vlink.h))实现控制功能，注释中大致标明了各函数的文件位置，~~不过大部分其实也一般不用改~~，您可能会接触到的几个函数包括:
```C++
void Vlink::keyboard_control_exp_set(void);
```
您可以查看或更改通过键盘的哪些键在仿真中遥控机器人。
```C++
void Vlink::console_print(void)
{
    cout << endl << count << endl;
    //Torque_print();
    //state_print(false , true, false);
    //virtual_model_print();
    //leg_print(rleg);
    //leg_print(lleg);
    //manipulator_print(Manipulator_L);
    //COM_print();
    //pid_print(turn_pid);
    //state_print(false, true, false);
    //lqr_print();
    //imu_print();
    //GPS_print();
    //file_print(0,0,0,lleg->Torque_b,lleg->Torque_f,lleg->T_foot);
    // file_print(state_l->test[0], state_l->test[1], state_l->test[2], state_l->test[3], state_l->test[4], state_l->test[5]);
    //file_print(gps_pos[0]->now(), gps_pos[2]->now(), path1(gps_pos[0]->now()), 0, 0, 0);
    //file_print(gps_pos[0]->now(),gps_pos[2]->now(),0,0,0,0);
    //
}
```
您可以解除感兴趣项的注释，并在webots console中得到对应项的输出。**特别的**，file_print()函数提供了一个将数据记录输出到excel表格的工具，便于进行模型训练或数据分析等。
~~如果您有大幅改动程序的需求，下面这段话或许有利于您理解程序~~
如果您有大幅改动程序的需求，还是来和我聊聊吧
### &#x1F60A;如果您想要导入新的模型
工程中提供了简化版的轮足机器人模型，您可以修改尺寸或导入新的轮足机器人模型，这大致需要以下三步：
1.参照[这篇文章](https://blog.csdn.net/weixin_45168199/article/details/105755388)的1~5步，利用Solidworks的SW2URDF插件导出urdf描述的机器人（**需要同时导出mesh**），并将生成的文件夹放到webots工程的proto文件夹下。
2.打开生成的文件夹的“urdf”子文件夹，并用命令行工具打开，使用webots官方提供的urdf2webotos工具，输入
```cmd
python -m urdf2webots.importer --input=myrobot.urdf
```
若没有安装此工具，参考[这篇文章](https://blog.csdn.net/weixin_45125306/article/details/118641926)。然而webots提供的这个工具~~很sb~~存在一些问题，生成的proto文件中，mesh的url地址常常有问题，而且是绝对路径，如果导入不了可以word打开proto文件检查一下，有时候url路径的“/”会变成“\”。
3.proto弄好了还远非万事大吉，以本工程的机器人节点为例，**导入proto后还需要做以下几件事：**
3.1 删除四个膝关节Hingejoint节点下device节点的RotationalMotor节点，因为五连杆髋关节是被动的
3.2删除前腿踝关节Hingejoint节点下的endpoint Solid节点，并添加endpoint SolidReference节点，选取Reference为后腿踝关节的endpoint Solid。因为sw2urdf并不支持并联关节的表示，所以1、2步导出的结构并不是并联结构，这里相当于利用SolidReference使得link“轮子”具有两个父节点，完成并联结构的表达。**同时也要记得删除前轮device节点的RotationalMotor节点**
3.3根据机器人具体情况设置关节的阻尼、摩擦等参数。并添加传感器。

**另一种更好的方法**：前文1、2步随便在什么位置导出都可以，直接把proto中mesh的路径改成相对路径：
```proto
从：
"D:/engineering/webots_projects/project0/protos/witharms/meshes/hip2thigh_lf.STL"
改成：
"meshes/hip2thigh_lf.STL"
```
之后把SW生成的meshes文件夹和改过的proto一起放到webots工程的proto文件夹下就可以了。
## 作者列表
