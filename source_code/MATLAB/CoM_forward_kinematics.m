%% Product of Exponentials (for V2 robot)
clear; clc; close all;
syms theta1 theta2 theta3 theta4 theta6 theta7 theta8 theta9 L1 L2 L3 L4 L5 real

% angular velocities
w1 = [0 0 1]';
w2 = [0 -1 0]';
w3 = [-1 0 0]';
w4 = [-1 0 0]';

w6 = [0 0 1]';
w7 = [0 -1 0]';
w8 = [-1 0 0]';
w9 = [-1 0 0]';

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

% end-effector frame configuration (basically the base frame)
M_left = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
M_right = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

% final transformation matrix
T_left = simplify(M_left*expm(S{4}*theta4)*expm(S{3}*theta3)*expm(S{2}*theta2)*expm(S{1}*theta1));
T_right = simplify(M_right*expm(S{9}*theta9)*expm(S{8}*theta8)*expm(S{7}*theta7)*expm(S{6}*theta6));
x_left = T_left(1,4)
y_left = T_left(2,4)
z_left = T_left(3,4)

x_right = T_right(1,4)
y_right = T_right(2,4)
z_right = T_right(3,4)

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

val_T_left = vpa(subs(T_left,sub_syms,sub_vals),3);
val_T_right = vpa(subs(T_right,sub_syms,sub_vals),3);

% forward kinematics equations for computation in Orin
% x_left = L1*cos(theta1)*cos(theta2) - L2*sin(theta2) - L1
% y_left = L4*sin(theta4) + L1*cos(theta3)*cos(theta4)*sin(theta1) + L2*cos(theta2)*cos(theta3)*sin(theta4) + L2*cos(theta2)*cos(theta4)*sin(theta3) - L1*sin(theta1)*sin(theta3)*sin(theta4) + L1*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + L1*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)
% z_left = L4*cos(theta4) - L4 - L2 + L2*cos(theta2)*cos(theta3)*cos(theta4) - L1*cos(theta3)*sin(theta1)*sin(theta4) - L1*cos(theta4)*sin(theta1)*sin(theta3) - L2*cos(theta2)*sin(theta3)*sin(theta4) + L1*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) - L1*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)
% x_right = L1 - L2*sin(theta7) - L1*cos(theta6)*cos(theta7)
% y_right = L4*sin(theta9) - L1*cos(theta8)*cos(theta9)*sin(theta6) + L2*cos(theta7)*cos(theta8)*sin(theta9) + L2*cos(theta7)*cos(theta9)*sin(theta8) + L1*sin(theta6)*sin(theta8)*sin(theta9) - L1*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) - L1*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8)
% z_right = L4*cos(theta9) - L4 - L2 + L2*cos(theta7)*cos(theta8)*cos(theta9) + L1*cos(theta8)*sin(theta6)*sin(theta9) + L1*cos(theta9)*sin(theta6)*sin(theta8) - L2*cos(theta7)*sin(theta8)*sin(theta9) - L1*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + L1*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9)