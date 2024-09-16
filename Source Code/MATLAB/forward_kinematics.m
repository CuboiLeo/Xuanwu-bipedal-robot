clear; clc; close all;

syms theta1 theta2 theta3 theta4;
syms a1 a2 a3 a4 a5 alpha2 alpha3;
% theta d a alpha
dhparams = [theta1 0 a1 0;
            theta2 0 a2 alpha2;
            theta3 0 a3 alpha3;
            theta4 0 a4 0;
            0 0 a5 0];
num_T_matrix = size(dhparams,1);
T = sym(zeros(4, 4, num_T_matrix));
T_05 = sym(eye(4));
for i = 1:5
    T(:,:,i) = [cos(dhparams (i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_05 = T_05*T(:,:,i);
end

T_05 = simplify(T_05);
T_05 = subs(T_05,{theta1 theta2 theta3 theta4 a1 a2 a3 a4 a5 alpha2 alpha3},{0 pi/2 0 0 -0.135 -0.095 -0.09 -0.18 -0.38 -pi/2 -pi/2})



