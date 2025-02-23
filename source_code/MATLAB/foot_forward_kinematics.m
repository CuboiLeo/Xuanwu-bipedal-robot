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
T_left = simplify(expand(expm(S_brkt{1}*theta{1})*expm(S_brkt{2}*theta{2})*expm(S_brkt{3}*theta{3})*expm(S_brkt{4}*theta{4})*expm(S_brkt{5}*theta{5})*M_left))
T_right = simplify(expm(S_brkt{6}*theta{6})*expm(S_brkt{7}*theta{7})*expm(S_brkt{8}*theta{8})*expm(S_brkt{9}*theta{9})*expm(S_brkt{10}*theta{10})*M_right)

%% The transformation matrixes are for Orin computation
% T_left =
% 
% [ cos(theta1)*cos(theta2), cos(theta5)*(cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2))) - sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3))), - cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3))) - sin(theta5)*(cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2))), L4*sin(theta1)*sin(theta3) - L1 - L4*cos(theta1)*cos(theta3)*sin(theta2) + L5*cos(theta3)*sin(theta1)*sin(theta4) + L5*cos(theta4)*sin(theta1)*sin(theta3) - L5*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + L6*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta5) + L6*cos(theta3)*cos(theta5)*sin(theta1)*sin(theta4) + L6*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta3) + L5*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4) - L6*sin(theta1)*sin(theta3)*sin(theta4)*sin(theta5) + L6*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)*sin(theta5) + L6*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)*sin(theta5) + L6*cos(theta1)*cos(theta5)*sin(theta2)*sin(theta3)*sin(theta4) - L6*cos(theta1)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)]
% [-cos(theta2)*sin(theta1), cos(theta5)*(cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) - sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) + sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3))), - cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) + sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3))) - sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))),      L4*cos(theta1)*sin(theta3) + L5*cos(theta1)*cos(theta3)*sin(theta4) + L5*cos(theta1)*cos(theta4)*sin(theta3) + L4*cos(theta3)*sin(theta1)*sin(theta2) + L6*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta5) + L6*cos(theta1)*cos(theta3)*cos(theta5)*sin(theta4) + L6*cos(theta1)*cos(theta4)*cos(theta5)*sin(theta3) + L5*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) - L6*cos(theta1)*sin(theta3)*sin(theta4)*sin(theta5) - L5*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4) + L6*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2) - L6*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4)*sin(theta5) - L6*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta5) - L6*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)]
% [            -sin(theta2),                                                                                                                                                                                                                                                                                                         sin(theta3 + theta4 + theta5)*cos(theta2),                                                                                                                                                                                                                                                                                                           cos(theta3 + theta4 + theta5)*cos(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                        L5*cos(theta2)*sin(theta3)*sin(theta4) - L4*cos(theta2)*cos(theta3) - L5*cos(theta2)*cos(theta3)*cos(theta4) - L2 - L6*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5) + L6*cos(theta2)*cos(theta3)*sin(theta4)*sin(theta5) + L6*cos(theta2)*cos(theta4)*sin(theta3)*sin(theta5) + L6*cos(theta2)*cos(theta5)*sin(theta3)*sin(theta4)]
% [                       0,                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            1]
% 
% 
% T_right =
% 
% [ cos(theta6)*cos(theta7), cos(theta10)*(cos(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7))) - sin(theta10)*(cos(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7)) + sin(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8))), - cos(theta10)*(cos(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7)) + sin(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8))) - sin(theta10)*(cos(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7))), L1 + L4*sin(theta6)*sin(theta8) - L4*cos(theta6)*cos(theta8)*sin(theta7) + L5*cos(theta8)*sin(theta6)*sin(theta9) + L5*cos(theta9)*sin(theta6)*sin(theta8) - L5*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + L6*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta10) + L6*cos(theta8)*cos(theta10)*sin(theta6)*sin(theta9) + L6*cos(theta9)*cos(theta10)*sin(theta6)*sin(theta8) + L5*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9) - L6*sin(theta6)*sin(theta8)*sin(theta9)*sin(theta10) + L6*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9)*sin(theta10) + L6*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8)*sin(theta10) + L6*cos(theta6)*cos(theta10)*sin(theta7)*sin(theta8)*sin(theta9) - L6*cos(theta6)*cos(theta8)*cos(theta9)*cos(theta10)*sin(theta7)]
% [-cos(theta7)*sin(theta6), cos(theta10)*(cos(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7))) - sin(theta10)*(cos(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7)) + sin(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8))), - cos(theta10)*(cos(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7)) + sin(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8))) - sin(theta10)*(cos(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7))),      L4*cos(theta6)*sin(theta8) + L5*cos(theta6)*cos(theta8)*sin(theta9) + L5*cos(theta6)*cos(theta9)*sin(theta8) + L4*cos(theta8)*sin(theta6)*sin(theta7) + L6*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta10) + L6*cos(theta6)*cos(theta8)*cos(theta10)*sin(theta9) + L6*cos(theta6)*cos(theta9)*cos(theta10)*sin(theta8) + L5*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) - L6*cos(theta6)*sin(theta8)*sin(theta9)*sin(theta10) - L5*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9) + L6*cos(theta8)*cos(theta9)*cos(theta10)*sin(theta6)*sin(theta7) - L6*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9)*sin(theta10) - L6*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta10) - L6*cos(theta10)*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9)]
% [            -sin(theta7),                                                                                                                                                                                                                                                                                                          sin(theta8 + theta9 + theta10)*cos(theta7),                                                                                                                                                                                                                                                                                                            cos(theta8 + theta9 + theta10)*cos(theta7),                                                                                                                                                                                                                                                                                                                                                                                                                            L5*cos(theta7)*sin(theta8)*sin(theta9) - L4*cos(theta7)*cos(theta8) - L5*cos(theta7)*cos(theta8)*cos(theta9) - L2 - L6*cos(theta7)*cos(theta8)*cos(theta9)*cos(theta10) + L6*cos(theta7)*cos(theta8)*sin(theta9)*sin(theta10) + L6*cos(theta7)*cos(theta9)*sin(theta8)*sin(theta10) + L6*cos(theta7)*cos(theta10)*sin(theta8)*sin(theta9)]
% [                       0,                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    1]