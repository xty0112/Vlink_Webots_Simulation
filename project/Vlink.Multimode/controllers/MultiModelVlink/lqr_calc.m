clear;
L0s=0.09:0.005:0.37; % L0鍙樺寲鑼冨洿
Ks=zeros(2,6,length(L0s)); % 瀛樻斁涓嶅悓L0瀵瑰簲鐨凨
count=0;

for step=1:length(L0s)
% 鎵?闇?绗﹀彿閲?
syms theta theta1 theta2 real; % theta1=dTheta, theta2=ddTheta
syms x x1 x2 real;
syms phi phi1 phi2 real;
syms T Tp N P Nm Pm Nf t real;
L=L0s(step)/2; Lm=L0s(step)/2;
Im = 0.019219844 ;
Iw = 0.001313775 ;
Ip = 0.011349285;
M=4.35;
mp=0.511;
mw=0.694;
l=0;
Radius=0.053;
g=9.8;
% 鏈哄櫒浜虹粨鏋勫弬鏁?
% R=0.02; L=L0s(step)/2; Lm=L0s(step)/2; l=0; mw=0.251327; mp=0.0248; M=0.325; Iw=5.02655e-05; Ip=(9.66667e-07+7.9975e-06)*2; Im=0.000338542;


% 杩涜鐗╃悊璁＄畻
Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1*theta1*sin(theta))-l*(phi2*cos(phi)-phi1*phi1*sin(phi1)));
Pm=M*g+M*((L+Lm)*(-theta1*theta1*cos(theta)-theta2*sin(theta))-l*(phi1*phi1*cos(phi)+phi2*sin(phi)));
N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1*theta1*sin(theta)));
P=Pm+mp*g+mp*L*(-theta1*theta1*cos(theta)-theta2*sin(theta));
count=count+1;
count
equ1=x2-(T-N*Radius)/(Iw/Radius+mw*Radius);
equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
count=count+1;
count
[x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
% 姹傚緱闆呭厠姣旂煩闃碉紝鐒跺悗寰楀埌鐘舵?佺┖闂存柟绋?
Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
A=vpa(subs(Ja,[theta theta1 x x1 phi phi1],[0 0 0 0 0 0]));
B=vpa(subs(Jb,[theta theta1 x x1 phi phi1],[0 0 0 0 0 0]));
count=count+1;
count
% 绂绘暎鍖?
T=0.008;
[G,H]=c2d(eval(A),eval(B),T);
count=count+1;
count
%看这里看这里看这里Q, R
test=1;
%权重矩阵参数
Q=diag([0.001 0.001 0.8 0.125 3 0.008]);
R=diag([1 0.01]);
% 姹傝В鍙嶉鐭╅樀K
Ks(:,:,step)=dlqr(G,H,Q,R);
end
count=count+1;
count
% 瀵筀鐨勬瘡涓厓绱犲叧浜嶭0杩涜鎷熷悎
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
count=count+1;
count
xlswrite('lqr_index.csv',index);
filename='lqr_index.txt';
fid= fopen(filename,'w');
for i=1:12
    fprintf(fid,'%s',"{");
    for j=1:4
        str=string(index(i,j));
        fprintf(fid,'%s',str);
        if j~= 4
            fprintf(fid,'%s',",");
        end
    end
    fprintf(fid,'%s\n',"},");
    count=count+1;
    count
end
fprintf(fid,'%s',"//Q=[");
for k=1:6
    str=string(Q(k,k))+" ";
    fprintf(fid,'%s',str);
end
fprintf(fid,'%s\n',"]");
str="//R=["+string(R(1,1))+" "+string(R(2,2))+"]";
fprintf(fid,'%s\n',str);
str="//T="+string(T);
fprintf(fid,'%s\n',str);
str="//RadiusOfWheel="+string(Radius);
fprintf(fid,'%s\n',str);
vpa(subs(K,L0,0.19),5)
count=count+1;
count