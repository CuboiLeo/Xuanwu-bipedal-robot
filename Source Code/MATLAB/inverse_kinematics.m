clear; close all; clc;
px = 0;
py = 0.001;
pz = -0.53999;
a2 = 0.095;
a3 = 0.09;
a4 = 0.18;
a5 = 0.38;

theta_1_1 = atan2(-py,-px)-atan2(sqrt(px^2+py^2-a2^2),a2)
theta_1_2 = pi+atan2(-py,-px)+atan2(-a2,sqrt(px^2+py^2-a2^2))
D = (px^2+py^2+pz^2-a2^2-a4^2-a5^2)/(2*a4*a5);
theta_4_1 = atan2(sqrt(1-D^2),D)
theta_4_2 = atan2(-sqrt(1-D^2),D)

theta_3_1 = atan2(py,sqrt(px^2+pz^2-a2^2))-atan2(a5*sin(theta_4_1),a4+a5*cos(theta_4_1))
theta_3_2 = atan2(py,sqrt(px^2+pz^2-a2^2))-atan2(a5*sin(theta_4_2),a4+a5*cos(theta_4_2))



