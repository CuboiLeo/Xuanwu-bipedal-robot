%% Product of Exponentials (for V2 robot)
clear; clc; close all;
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10 L1 L2 L3 L4 L5 L6 real

% angular velocities
w1 = [0 0 -1]';
w2 = [0 1 0]';
w3 = [1 0 0]';
w4 = [1 0 0]';
w5 = [1 0 0]';
w6 = [0 0 -1]';
w7 = [0 1 0]';
w8 = [1 0 0]';
w9 = [1 0 0]';
w10 = [1 0 0]';

% point on the axis
q1 = [-L1 0 0]';
q2 = [-L1 -L3 -L2]';
q3 = [-L1 0 -L2]';
q4 = [-L1 0 -(L2+L4)]';
q5 = [-L1 0 -(L2+L4+L5)]';
q6 = [L1 0 0]';
q7 = [L1 -L3 -L2]';
q8 = [L1 0 -L2]';
q9 = [L1 0 -(L2+L4)]';
q10 = [L1 0 -(L2+L4+L5)]';

% linear velocity
v1 = -cross(w1,q1);
v2 = -cross(w2,q2);
v3 = -cross(w3,q3);
v4 = -cross(w4,q4);
v5 = -cross(w5,q5);
v6 = -cross(w6,q6);
v7 = -cross(w7,q7);
v8 = -cross(w8,q8);
v9 = -cross(w9,q9);
v10 = -cross(w10,q10);

% screw axes
W = {w1,w2,w3,w4,w5,w6,w7,w8,w9,w10};
V = {v1,v2,v3,v4,v5,v6,v7,v8,v9,v10};
theta = {theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8,theta9,theta10};
for i = 1:10
    S{i} = [W{i};V{i}];
    S_brkt{i} = [0 -W{i}(3) W{i}(2) V{i}(1);
        W{i}(3) 0 -W{i}(1) V{i}(2);
        -W{i}(2) W{i}(1) 0 V{i}(3);
        0         0   0    0   ];
end

% end-effector frame configuration
M_left = [1 0 0 -L1;
    0 1 0 0;
    0 0 1 -(L2+L4+L5+L6);
    0 0 0 1];
M_right = [1 0 0 L1;
    0 1 0 0;
    0 0 1 -(L2+L4+L5+L6);
    0 0 0 1];

% final transformation matrix
T_left = simplify(expm(S_brkt{1}*theta{1})*expm(S_brkt{2}*theta{2})*expm(S_brkt{3}*theta{3})*expm(S_brkt{4}*theta{4})*expm(S_brkt{5}*theta{5})*M_left);
T_right = simplify(expm(S_brkt{6}*theta{6})*expm(S_brkt{7}*theta{7})*expm(S_brkt{8}*theta{8})*expm(S_brkt{9}*theta{9})*expm(S_brkt{10}*theta{10})*M_right);

% calculate space Jacobian
for i = 1:5
    if i == 1
        Js_left(:,1) = S{1};
        Js_right(:,1) = S{6};
    else
        T_l = eye(4);
        T_r = eye(4);
        for j = 2:i
            T_l = T_l*expm(S_brkt{j-1}*theta{j-1});
        end
        for k = 7:(i+5)
            T_r = T_r*expm(S_brkt{k-1}*theta{k-1});
        end
        Ad_T_left = adj_rep(T_l);
        Js_left(:,i) = Ad_T_left*S{i};
        Ad_T_right = adj_rep(T_r);
        Js_right(:,i) = Ad_T_right*S{i+5};
    end
end
Js_left = simplify(Js_left);
Js_right = simplify(Js_right);

% value substitution
val_theta1 = 0.001;
val_theta2 = 0.001;
val_theta3 = 0.3;
val_theta4 = -0.3;
val_theta5 = 0;
val_theta6 = 0;
val_theta7 = 0;
val_theta8 = 0;
val_theta9 = 0;
val_theta10 = 0;
val_L1 = 0.135;
val_L2 = 0.12;
val_L3 = 0.09;
val_L4 = 0.18;
val_L5 = 0.18;
val_L6 = 0.075;
sub_vals = [val_L1 val_L2 val_L3 val_L4 val_L5 val_L6]';
sub_syms = [L1 L2 L3 L4 L5 L6]';
val_theta = [val_theta1 val_theta2 val_theta3 val_theta4 val_theta5]';
sub_theta = [theta1 theta2 theta3 theta4 theta5]';

% desired end effector transformation
euler = [pi/9 0 0];
rotmZYX = eul2rotm(euler);
p_d = [-0.135 0.01 -0.53];
Tsd = eye(4);
Tsd(1:3,1:3) = rotmZYX;
Tsd(1:3,4) = p_d;

% Newton Raphson Method
norm_ws = 1;
norm_vs = 1;
epsilon_w = 0.005;
epsilon_v = 0.005;
iteration = 0;

while norm_ws > epsilon_w || norm_vs > epsilon_v && iteration < 100
    Tsb = vpa(subs(T_left,[sub_theta;sub_syms],[val_theta;sub_vals]),3);
    Tbd = inv(Tsb)*Tsd;
    Vb_brkt = logm(Tbd);
    Vb = [Vb_brkt(3,2);Vb_brkt(1,3);Vb_brkt(2,1);Vb_brkt(1,4);Vb_brkt(2,4);Vb_brkt(3,4)];
    Vs = adj_rep(Tsb)*Vb;
    Js = vpa(subs(Js_left,[sub_theta;sub_syms],[val_theta;sub_vals]),3);
    val_theta = real(val_theta + pinv(Js)*Vs)

    norm_ws = norm(Vs(1:3,1));
    norm_vs = norm(Vs(4:6,1));
    iteration = iteration + 1;
end

%% The Jacobian matrixes are for Orin computation
% Js_left =
% 
% [  0,     sin(theta1),                       cos(theta1)*cos(theta2),                                                                                           cos(theta1)*cos(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               cos(theta1)*cos(theta2)]
% [  0,     cos(theta1),                      -cos(theta2)*sin(theta1),                                                                                          -cos(theta2)*sin(theta1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -cos(theta2)*sin(theta1)]
% [ -1,               0,                                  -sin(theta2),                                                                                                      -sin(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(theta2)]
% [  0,  L2*cos(theta1),                   -L2*cos(theta2)*sin(theta1),                - L2*cos(theta2)*sin(theta1) - L4*cos(theta3)*sin(theta1) - L4*cos(theta1)*sin(theta2)*sin(theta3), sin(theta2)*(sin(theta1)*(L1*cos(theta2) - L1 + L2*sin(theta2)) + L1*sin(theta1) + (cos(theta4) - 1)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))*(L2 + L4) + L2*cos(theta1)*sin(theta3) + sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3))*(L2 + L4) + L2*sin(theta1)*sin(theta2)*(cos(theta3) - 1)) - (L2 + L4 + L5)*(cos(theta3)*cos(theta4)*sin(theta1) - sin(theta1)*sin(theta3)*sin(theta4) + cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)) - cos(theta2)*sin(theta1)*(L2 + L1*sin(theta2) + L4*cos(theta2)*cos(theta3) - L2*cos(theta2)*cos(theta3)*cos(theta4) - L4*cos(theta2)*cos(theta3)*cos(theta4) + L2*cos(theta2)*sin(theta3)*sin(theta4) + L4*cos(theta2)*sin(theta3)*sin(theta4))]
% [-L1, -L2*sin(theta1), - L1*sin(theta2) - L2*cos(theta1)*cos(theta2), L4*sin(theta1)*sin(theta2)*sin(theta3) - L2*cos(theta1)*cos(theta2) - L4*cos(theta1)*cos(theta3) - L1*sin(theta2),              - (cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))*(L2 + L4 + L5) - sin(theta2)*(L1 - cos(theta1)*(L1*cos(theta2) - L1 + L2*sin(theta2)) - L1*cos(theta1) + (cos(theta4) - 1)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2))*(L2 + L4) + L2*sin(theta1)*sin(theta3) + sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3))*(L2 + L4) - L2*cos(theta1)*sin(theta2)*(cos(theta3) - 1)) - cos(theta1)*cos(theta2)*(L2 + L1*sin(theta2) + L4*cos(theta2)*cos(theta3) - L2*cos(theta2)*cos(theta3)*cos(theta4) - L4*cos(theta2)*cos(theta3)*cos(theta4) + L2*cos(theta2)*sin(theta3)*sin(theta4) + L4*cos(theta2)*sin(theta3)*sin(theta4))]
% [  0, -L1*cos(theta1),                    L1*cos(theta2)*sin(theta1),                                                                     cos(theta2)*(L1*sin(theta1) - L4*sin(theta3)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -cos(theta2)*(L5*sin(theta3 + theta4) - L1*sin(theta1) + L4*sin(theta3))]
% 
% 
% Js_right =
% 
% [ 0,     sin(theta6),                     cos(theta6)*cos(theta7),                                                                                           cos(theta6)*cos(theta7),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               cos(theta6)*cos(theta7)]
% [ 0,     cos(theta6),                    -cos(theta7)*sin(theta6),                                                                                          -cos(theta7)*sin(theta6),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -cos(theta7)*sin(theta6)]
% [-1,               0,                                -sin(theta7),                                                                                                      -sin(theta7),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(theta7)]
% [ 0,  L2*cos(theta6),                 -L2*cos(theta7)*sin(theta6),                - L2*cos(theta7)*sin(theta6) - L4*cos(theta8)*sin(theta6) - L4*cos(theta6)*sin(theta7)*sin(theta8), sin(theta7)*(sin(theta6)*(L1 - L1*cos(theta7) + L2*sin(theta7)) - L1*sin(theta6) + (cos(theta9) - 1)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7))*(L2 + L4) + L2*cos(theta6)*sin(theta8) + sin(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8))*(L2 + L4) + L2*sin(theta6)*sin(theta7)*(cos(theta8) - 1)) - (L2 + L4 + L5)*(cos(theta8)*cos(theta9)*sin(theta6) - sin(theta6)*sin(theta8)*sin(theta9) + cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8)) - cos(theta7)*sin(theta6)*(L2 - L1*sin(theta7) + L4*cos(theta7)*cos(theta8) - L2*cos(theta7)*cos(theta8)*cos(theta9) - L4*cos(theta7)*cos(theta8)*cos(theta9) + L2*cos(theta7)*sin(theta8)*sin(theta9) + L4*cos(theta7)*sin(theta8)*sin(theta9))]
% [L1, -L2*sin(theta6), L1*sin(theta7) - L2*cos(theta6)*cos(theta7), L1*sin(theta7) - L2*cos(theta6)*cos(theta7) - L4*cos(theta6)*cos(theta8) + L4*sin(theta6)*sin(theta7)*sin(theta8),              - (cos(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7)))*(L2 + L4 + L5) - sin(theta7)*(L1*cos(theta6) - L1 - cos(theta6)*(L1 - L1*cos(theta7) + L2*sin(theta7)) + (cos(theta9) - 1)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7))*(L2 + L4) + L2*sin(theta6)*sin(theta8) + sin(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8))*(L2 + L4) - L2*cos(theta6)*sin(theta7)*(cos(theta8) - 1)) - cos(theta6)*cos(theta7)*(L2 - L1*sin(theta7) + L4*cos(theta7)*cos(theta8) - L2*cos(theta7)*cos(theta8)*cos(theta9) - L4*cos(theta7)*cos(theta8)*cos(theta9) + L2*cos(theta7)*sin(theta8)*sin(theta9) + L4*cos(theta7)*sin(theta8)*sin(theta9))]
% [ 0,  L1*cos(theta6),                 -L1*cos(theta7)*sin(theta6),                                                                    -cos(theta7)*(L1*sin(theta6) + L4*sin(theta8)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -cos(theta7)*(L5*sin(theta8 + theta9) + L1*sin(theta6) + L4*sin(theta8))]
