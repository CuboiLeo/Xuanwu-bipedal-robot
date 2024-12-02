%% Product of Exponentials (for V2 robot)
clear; clc; close all;
syms theta1 theta2 theta3 theta4 theta6 theta7 theta8 theta9 L1 L2 L3 L4 L5 real

% angular velocities
w1 = [0 0 -1]';
w2 = [0 1 0]';
w3 = [1 0 0]';
w4 = [1 0 0]';

w6 = [0 0 -1]';
w7 = [0 1 0]';
w8 = [1 0 0]';
w9 = [1 0 0]';

% point on the axis
q1 = [-L1 0 0]';
q2 = [-L1 -L3 -L2]';
q3 = [-L1 0 -L2]';
q4 = [-L1 0 -(L2+L4)]';

q6 = [L1 0 0]';
q7 = [L1 -L3 -L2]';
q8 = [L1 0 -L2]';
q9 = [L1 0 -(L2+L4)]';

% linear velocity
v1 = -cross(w1,q1);
v2 = -cross(w2,q2);
v3 = -cross(w3,q3);
v4 = -cross(w4,q4);

v6 = -cross(w6,q6);
v7 = -cross(w7,q7);
v8 = -cross(w8,q8);
v9 = -cross(w9,q9);

% screw axes
W = {w1,w2,w3,w4,0,w6,w7,w8,w9};
V = {v1,v2,v3,v4,0,v6,v7,v8,v9};
for i = [1:4,6:9]
    S{i} = [0 -W{i}(3) W{i}(2) V{i}(1);
            W{i}(3) 0 -W{i}(1) V{i}(2);
            -W{i}(2) W{i}(1) 0 V{i}(3);
               0         0   0    0   ];
end

% end-effector frame configuration
M_left = [1 0 0 -L1;
     0 1 0 0;
     0 0 1 -(L2+L4+L5);
     0 0 0 1];
M_right = [1 0 0 L1;
     0 1 0 0;
     0 0 1 -(L2+L4+L5);
     0 0 0 1];
% final transformation matrix
T_left = expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*expm(S{4}*theta4)*M_left;
T_right = expm(S{6}*theta6)*expm(S{7}*theta7)*expm(S{8}*theta8)*expm(S{9}*theta9)*M_right;

% value substitution
val_theta1 = 0;
val_theta2 = pi/2;
val_theta3 = 0;
val_theta4 = 0;
val_theta6 = 0;
val_theta7 = pi/2;
val_theta8 = 0;
val_theta9 = 0;
val_L1 = 0.135;
val_L2 = 0.12;
val_L3 = 0.09;
val_L4 = 0.18;
val_L5 = 0.27;
sub_vals = [val_theta1 val_theta2 val_theta3 val_theta4 val_theta6 val_theta7 val_theta8 val_theta9 val_L1 val_L2 val_L3 val_L4 val_L5];
sub_syms = [theta1 theta2 theta3 theta4 theta6 theta7 theta8 theta9 L1 L2 L3 L4 L5];

val_T_left = vpa(subs(T_left,sub_syms,sub_vals),2)
val_T_right = vpa(subs(T_right,sub_syms,sub_vals),2)

%% Modified DH Method (for V1 robot)
clear; clc; close all;
syms theta1 theta2 theta3 theta4;
syms a1 a2 a3 a4 a5 alpha2 alpha3;
% theta d a alpha
dhparams = [theta1 0 a1 0;
            theta2 0 a2 alpha2;
            theta3 0 a3 alpha3;
            theta4 0 a4 0;
            0      0 a5 0];
num_T_matrix = size(dhparams,1);
T = sym(zeros(4, 4, num_T_matrix));
T_05 = sym(eye(4));
for i = 1:5
    T(:,:,i) = [cos(dhparams(i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_05 = T_05*T(:,:,i);
end

T_05 = simplify(T_05);
x = T_05(1,4);
y = T_05(2,4);
z = T_05(3,4);
yaw = atan2(T_05(2,1), T_05(1,1));

sym_theta = [theta1 theta2 theta3 theta4];
val_theta = [   -0.7351  1.5811 -0.5009 0.5051]; 
sym_sub = [a1 a2 a3 a4 a5 alpha2 alpha3];
val_sub = [-0.135 -0.095 -0.09 -0.18 -0.38 -pi/2 -pi/2];

val_x = vpa(subs(x,[sym_theta sym_sub],[val_theta val_sub]),2)
val_y = vpa(subs(y,[sym_theta sym_sub],[val_theta val_sub]),2)
val_z = vpa(subs(z,[sym_theta sym_sub],[val_theta val_sub]),2)
val_yaw = vpa(subs(yaw,[sym_theta sym_sub],[val_theta val_sub]),2)
%T_05 = subs(T_05,{theta1 theta2 theta3 theta4 a1 a2 a3 a4 a5 alpha2 alpha3},{0 pi/2 0 0 -0.135 -0.095 -0.09 -0.18 -0.38 -pi/2 -pi/2})
