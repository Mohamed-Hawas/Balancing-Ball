clc
m = 0.0065;
R = 0.034;
g = -9.8; 
L = 0.302;
d = 0.035;
J = 3.0056e-6;

s = tf('s');
P_ball = -m*g*d/L/(J/R^2+m)/s^2

%pzmap(P_ball);
%step(P_ball);

Kp = 2.5;
Ki = 1 ;
Kd = 1.7;


C = pid(Kp,Ki,Kd);

sys_cl=feedback(C*P_ball,1);

pidtool(sys_cl);

%t=0:0.01:5;
step(sys_cl)