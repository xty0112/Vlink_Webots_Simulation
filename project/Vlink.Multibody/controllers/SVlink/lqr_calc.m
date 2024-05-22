% 本程序用于求解LQR反馈矩阵lqr_k(L0)
% 对于每一个腿长L0，求解一次系统状态空间方程，然后求得反馈矩阵K
% 对于不同的K，对L0进行拟合，得到lqr_k

%clear;

L0s=0.12:0.01:0.30; % L0变化范围
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K

for step=1:length(L0s)
    
    % 所需符号量
    syms theta theta1 theta2 real; % theta1=dTheta, theta2=ddTheta
    syms x x1 x2 real;
    syms phi phi1 phi2 real;
    syms T Tp N P Nm Pm Nf t M mw R L Lm l mw mp Iw Ip Im g real;
    
    % 机器人结构参数
%[    R,        L,        Lm,     l,    mw,   mp?,   M,       Iw,    Ip?,     Im,     g]
Ind=[0.053 L0s(step)/2 L0s(step)/2 0 0.522 0.248 4.564 7.33e-04 2*8.9975e-05 0.037678 9.8];
%R=0.02; L=L0s(step)/2; Lm=L0s(step)/2; l=0; mw=0.251327; mp=0.0248; M=0.325; Iw=5.02655e-05; Ip=(9.66667e-07+7.9975e-06)*2; Im=0.000338542;
%g=9.8;

sin2theta=theta2*cos(theta)-theta1*theta1*sin(theta);
cos2theta=-theta2*sin(theta)-theta1*theta1*cos(theta);
sin2phi=phi2*cos(phi)-phi1*phi1*sin(phi);
cos2phi=-phi2*sin(phi)-phi1*phi1*cos(phi);
%中间变量
Nm=M*x2+M*(L+Lm)*sin2theta-M*l*sin2phi;
Pm=M*g+M*(L+Lm)*cos2theta+M*l*cos2phi;
N=Nm+mp*(x2+L*sin2theta);
P=Pm+mp*g+mp*L*cos2theta;
%关系
eqn1=(Iw+mw*R*R)*x2-T*R+N*R*R;
eqn2=-Ip*theta2+(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp;
eqn3=-Im*phi2+Tp+Nm*l*cos(phi)+Pm*l*sin(phi);
[x2,theta2,phi2]=solve(eqn1,eqn2,eqn3,x2,theta2,phi2);
%求平衡点处雅可比矩阵
Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
A=subs(Ja,[theta,theta1,phi,phi1,x1],zeros(1,5));
Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
B=subs(Jb,[theta,theta1,phi,phi1,x1],zeros(1,5));
A=subs(A,[R,L,Lm,l,mw,mp,M,Iw, Ip, Im, g],Ind);
B=subs(B,[R,L,Lm,l,mw,mp,M,Iw, Ip, Im, g],Ind);
    % 离散化
    A=eval(A);
    B=eval(B);
    T=0.032;
    [G,H]=c2d(A,B,T);
    power=[1 1 500 100 5000 10];
    % 定义权重矩阵Q, R
    Q=diag(power);
    R=diag([1 0.25]);

    % 求解反馈矩阵K
    Ks(:,:,step)=dlqr(G,H,Q,R);

end

% 对K的每个元素关于L0进行拟合
K=sym('K',[2 6]);
syms L0;
index=zeros(12,4);
for x=1:2
    for y=1:6
        p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        for i=1:4
            index(2*y+x-2,i)=p(i);
        end
        K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end
xlswrite('lqr_index.csv',index);
% 输出到m函数
%matlabFunction(K,'File','lqr_k');

% 代入L0=0.07打印矩阵K
vpa(subs(K,L0,0.15),5)
power
T
