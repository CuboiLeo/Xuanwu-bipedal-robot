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

% value substitution
val_theta1 = 0;
val_theta2 = 0.0001;
val_theta3 = 0.0001;
val_theta4 = 0.0001;
val_theta6 = 0;
val_theta7 = 0.0001;
val_theta8 = 0.0001;
val_theta9 = 0.0001;
val_L1 = 0.135;
val_L2 = 0.12;
val_L3 = 0.09;
val_L4 = 0.18;
val_L5 = 0.27;
sub_vals = [val_theta1 val_theta6  val_L1 val_L2 val_L3 val_L4 val_L5];
sub_syms = [theta1 theta6 L1 L2 L3 L4 L5];

sym_theta_left = [theta2 theta3 theta4];
val_theta_left = [val_theta2 val_theta3 val_theta4];
sym_theta_right = [theta7 theta8 theta9];
val_theta_right = [val_theta7 val_theta8 val_theta9];

% coordinate vector
x_left = T_left(1,4);
y_left = T_left(2,4);
z_left = T_left(3,4);
sym_X_act_left = [x_left;y_left;z_left];

x_right = T_right(1,4);
y_right = T_right(2,4);
z_right = T_right(3,4);
sym_X_act_right = [x_right;y_right;z_right];

X_ref = [-0.02;0;-0.01];

norm_e_left = 1;
norm_e_right = 1; 
epsilon = 0.005;
iteration = 0;
jacob_left = simplify(jacobian(sym_X_act_left,sym_theta_left));
jacob_right = simplify(jacobian(sym_X_act_right,sym_theta_right));

while (norm_e_left > epsilon || norm_e_right > epsilon) && iteration < 100
    val_X_act_left = vpa(subs(sym_X_act_left,[sym_theta_left sub_syms],[val_theta_left sub_vals]),8)
    val_X_act_right = vpa(subs(sym_X_act_right,[sym_theta_right sub_syms],[val_theta_right sub_vals]),8)

    error_X_left = vpa(X_ref - val_X_act_left,8);
    error_X_right = vpa(X_ref - val_X_act_right,8);

    norm_e_left = norm(error_X_left);
    norm_e_right = norm(error_X_right);

    val_jacob_left = vpa(subs(jacob_left,[sym_theta_left sub_syms],[val_theta_left sub_vals]),8);
    val_jacob_right = vpa(subs(jacob_right,[sym_theta_right sub_syms],[val_theta_right sub_vals]),8);

    inv_jacob_left = pinv(val_jacob_left);
    inv_jacob_right = pinv(val_jacob_right);

    delta_theta_left = inv_jacob_left*error_X_left;
    delta_theta_right = inv_jacob_right*error_X_right;

    val_theta_left = val_theta_left + delta_theta_left';
    val_theta_left = mod(eval(val_theta_left) + pi, 2*pi) - pi;

    val_theta_right = val_theta_right + delta_theta_right';
    val_theta_right = mod(eval(val_theta_right) + pi, 2*pi) - pi;

    iteration = iteration + 1;
end

val_theta_left
val_theta_right

% CoM left leg Jacobian matrix for Orin
% jacob(1,1) = - L2*cos(theta2) - L1*cos(theta1)*sin(theta2)
% jacob(1,2) = 0
% jacob(1,3) = 0
% jacob(2,1) = -sin(theta3 + theta4)*(L2*sin(theta2) - L1*cos(theta1)*cos(theta2))
% jacob(2,2) = L2*cos(theta2)*cos(theta3)*cos(theta4) - L1*cos(theta3)*sin(theta1)*sin(theta4) - L1*cos(theta4)*sin(theta1)*sin(theta3) - L2*cos(theta2)*sin(theta3)*sin(theta4) + L1*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) - L1*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)
% jacob(2,3) = L4*cos(theta4) + L2*cos(theta2)*cos(theta3)*cos(theta4) - L1*cos(theta3)*sin(theta1)*sin(theta4) - L1*cos(theta4)*sin(theta1)*sin(theta3) - L2*cos(theta2)*sin(theta3)*sin(theta4) + L1*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) - L1*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)
% jacob(3,1) = -cos(theta3 + theta4)*(L2*sin(theta2) - L1*cos(theta1)*cos(theta2))
% jacob(3,2) = L1*sin(theta1)*sin(theta3)*sin(theta4) - L2*cos(theta2)*cos(theta3)*sin(theta4) - L2*cos(theta2)*cos(theta4)*sin(theta3) - L1*cos(theta3)*cos(theta4)*sin(theta1) - L1*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) - L1*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)
% jacob(3,3) = L1*sin(theta1)*sin(theta3)*sin(theta4) - L1*cos(theta3)*cos(theta4)*sin(theta1) - L2*cos(theta2)*cos(theta3)*sin(theta4) - L2*cos(theta2)*cos(theta4)*sin(theta3) - L4*sin(theta4) - L1*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) - L1*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)

% CoM right leg Jacobian matrix for Orin
% jacob(1,1) = L1*cos(theta6)*sin(theta7) - L2*cos(theta7)
% jacob(1,2) = 0
% jacob(1,3) = 0
% jacob(2,1) = -sin(theta8 + theta9)*(L2*sin(theta7) + L1*cos(theta6)*cos(theta7))
% jacob(2,2) = L2*cos(theta7)*cos(theta8)*cos(theta9) + L1*cos(theta8)*sin(theta6)*sin(theta9) + L1*cos(theta9)*sin(theta6)*sin(theta8) - L2*cos(theta7)*sin(theta8)*sin(theta9) - L1*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + L1*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9)
% jacob(2,3) = L4*cos(theta9) + L2*cos(theta7)*cos(theta8)*cos(theta9) + L1*cos(theta8)*sin(theta6)*sin(theta9) + L1*cos(theta9)*sin(theta6)*sin(theta8) - L2*cos(theta7)*sin(theta8)*sin(theta9) - L1*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + L1*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9)
% jacob(3,1) = -cos(theta8 + theta9)*(L2*sin(theta7) + L1*cos(theta6)*cos(theta7))
% jacob(3,2) = L1*cos(theta8)*cos(theta9)*sin(theta6) - L2*cos(theta7)*cos(theta8)*sin(theta9) - L2*cos(theta7)*cos(theta9)*sin(theta8) - L1*sin(theta6)*sin(theta8)*sin(theta9) + L1*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + L1*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8)
% jacob(3,3) = L1*cos(theta8)*cos(theta9)*sin(theta6) - L4*sin(theta9) - L2*cos(theta7)*cos(theta8)*sin(theta9) - L2*cos(theta7)*cos(theta9)*sin(theta8) - L1*sin(theta6)*sin(theta8)*sin(theta9) + L1*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + L1*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8)
