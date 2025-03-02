%% for V2 robot
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
for i = 1:10
    S{i} = [0 -W{i}(3) W{i}(2) V{i}(1);
        W{i}(3) 0 -W{i}(1) V{i}(2);
        -W{i}(2) W{i}(1) 0 V{i}(3);
        0         0   0    0   ];
end

% substitute values
val_theta1 = 0;
val_theta2 = 0;
val_theta3 = pi/6;
val_theta4 = -pi/8;
val_theta5 = 0;
val_theta6 = 0;
val_theta7 = 0;
val_theta8 = pi/6;
val_theta9 = -pi/8;
val_theta10 = 0;
val_L1 = 0.135;
val_L2 = 0.12;
val_L3 = 0.09;
val_L4 = 0.18;
val_L5 = 0.18;
val_L6 = 0.075;

m1 = 1.73;
m2 = 0.55;
m3 = 0.38;
m4 = 0.53;
m5 = 0.53;
m6 = 0.14;

% link's relative offset from the last joint
syms C11 C12 C13 C21 C22 C23 C31 C32 C33 C41 C42 C43 C51 C52 C53 C61 C62 C63
C1 = [C11 C12 C13];
C2 = [C21 C22 C23];
C3 = [C31 C32 C33];
C4 = [C41 C42 C43];
C5 = [C51 C52 C53];
C6 = [C61 C62 C63];
val_C1 = [0 0 0.027];
val_C2 = [-0.0075 -0.06 -0.067];
val_C3 = [0 0.084 0];
val_C4 = [0 0 -0.142];
val_C5 = [0 0 -0.142];
val_C6 = [0 0 -0.06];

sub_vals = [val_theta1 val_theta2 val_theta3 val_theta4 val_theta5 val_theta6 val_theta7...
    val_theta8 val_theta9 val_theta10 val_L1 val_L2 val_L3 val_L4 val_L5 val_L6 val_C1 val_C2 val_C3 val_C4 val_C5 val_C6];
sub_syms = [theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10 L1 L2 L3 L4 L5 L6 C1 C2 C3 C4 C5 C6];

M1 = [1 0 0 C1(1);
    0 1 0 C1(2);
    0 0 1 C1(3);
    0 0 0 1];
M2L = [1 0 0 -L1+C2(1);
    0 1 0 C2(2);
    0 0 1 C2(3);
    0 0 0 1];
M3L = [1 0 0 -L1+C3(1);
    0 1 0 -L3+C3(2);
    0 0 1 -L2+C3(3);
    0 0 0 1];
M4L = [1 0 0 -L1+C4(1);
    0 1 0 C4(2);
    0 0 1 -L2+C4(3);
    0 0 0 1];
M5L = [1 0 0 -L1+C5(1);
    0 1 0 C5(2);
    0 0 1 -(L2+L4)+C5(3);
    0 0 0 1];
M6L = [1 0 0 -L1+C6(1);
    0 1 0 C6(2);
    0 0 1 -(L2+L4+L5)+C6(3);
    0 0 0 1];
M2R = [1 0 0 L1-C2(1);
    0 1 0 C2(2);
    0 0 1 C2(3);
    0 0 0 1];
M3R = [1 0 0 L1+C3(1);
    0 1 0 -L3+C3(2);
    0 0 1 -L2+C3(3);
    0 0 0 1];
M4R = [1 0 0 L1+C4(1);
    0 1 0 C4(2);
    0 0 1 -L2+C4(3);
    0 0 0 1];
M5R = [1 0 0 L1+C5(1);
    0 1 0 C5(2);
    0 0 1 -(L2+L4)+C5(3);
    0 0 0 1];
M6R = [1 0 0 L1+C6(1);
    0 1 0 C6(2);
    0 0 1 -(L2+L4+L5)+C6(3);
    0 0 0 1];

T1 = M1;
T2L = simplify(expm(S{1}*theta1)*M2L)
T3L = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*M3L);
T4L = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*M4L);
T5L = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*expm(S{4}*theta4)*M5L);
T6L = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*expm(S{4}*theta4)*expm(S{5}*theta5)*M6L);

T2R = simplify(expm(S{6}*theta6)*M2R);
T3R = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*M3R);
T4R = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*expm(S{8}*theta8)*M4R);
T5R = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*expm(S{8}*theta8)*expm(S{9}*theta9)*M5R);
T6R = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*expm(S{8}*theta8)*expm(S{9}*theta9)*expm(S{10}*theta10)*M6R);

T = {T1,T2L,T3L,T4L,T5L,T6L,T2R,T3R,T4R,T5R,T6R};
m = {m1,m2,m3,m4,m5,m6,m2,m3,m4,m5,m6};

% CoM calculation
x_total = 0;
y_total = 0;
z_total = 0;
m_total = 0;
for i = 1:11
    x_total = x_total + T{i}(1,4)*m{i};
    y_total = y_total + T{i}(2,4)*m{i};
    z_total = z_total + T{i}(3,4)*m{i};
    m_total = m_total + m{i};
end
x_total = x_total/m_total;
y_total = y_total/m_total;
z_total = z_total/m_total;

pos = vpa(subs([x_total;y_total;z_total],sub_syms,sub_vals),3)

%% The transformation matrixes are for Orin computation
% T1 =
% 
% [1, 0, 0, C11]
% [0, 1, 0, C12]
% [0, 0, 1, C13]
% [0, 0, 0,   1]
% 
% 
% T2L =
% 
% [ cos(theta1), sin(theta1), 0, C21*cos(theta1) - L1 + C22*sin(theta1)]
% [-sin(theta1), cos(theta1), 0,      C22*cos(theta1) - C21*sin(theta1)]
% [           0,           0, 1,                                    C23]
% [           0,           0, 0,                                      1]
% 
% 
% T3L =
% 
% [ cos(theta1)*cos(theta2), sin(theta1),  cos(theta1)*sin(theta2), C32*sin(theta1) - L1 - L3*sin(theta1) + C31*cos(theta1)*cos(theta2) + C33*cos(theta1)*sin(theta2)]
% [-cos(theta2)*sin(theta1), cos(theta1), -sin(theta1)*sin(theta2),      C32*cos(theta1) - L3*cos(theta1) - C31*cos(theta2)*sin(theta1) - C33*sin(theta1)*sin(theta2)]
% [            -sin(theta2),           0,              cos(theta2),                                                            C33*cos(theta2) - L2 - C31*sin(theta2)]
% [                       0,           0,                        0,                                                                                                 1]
% 
% 
% T4L =
% 
% [ cos(theta1)*cos(theta2), cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3),   cos(theta1)*cos(theta3)*sin(theta2) - sin(theta1)*sin(theta3), C41*cos(theta1)*cos(theta2) - L1 + C42*cos(theta3)*sin(theta1) - C43*sin(theta1)*sin(theta3) + C43*cos(theta1)*cos(theta3)*sin(theta2) + C42*cos(theta1)*sin(theta2)*sin(theta3)]
% [-cos(theta2)*sin(theta1), cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3), - cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2),      C42*cos(theta1)*cos(theta3) - C41*cos(theta2)*sin(theta1) - C43*cos(theta1)*sin(theta3) - C43*cos(theta3)*sin(theta1)*sin(theta2) - C42*sin(theta1)*sin(theta2)*sin(theta3)]
% [            -sin(theta2),                                       cos(theta2)*sin(theta3),                                         cos(theta2)*cos(theta3),                                                                                                 C43*cos(theta2)*cos(theta3) - C41*sin(theta2) - L2 + C42*cos(theta2)*sin(theta3)]
% [                       0,                                                             0,                                                               0,                                                                                                                                                                                1]
% 
% 
% T5L =
% 
% [ cos(theta1)*cos(theta2), cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)), - cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)), C51*cos(theta1)*cos(theta2) - L1 + L4*sin(theta1)*sin(theta3) + C52*cos(theta3)*cos(theta4)*sin(theta1) - L4*cos(theta1)*cos(theta3)*sin(theta2) - C53*cos(theta3)*sin(theta1)*sin(theta4) - C53*cos(theta4)*sin(theta1)*sin(theta3) - C52*sin(theta1)*sin(theta3)*sin(theta4) + C53*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C52*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C52*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) - C53*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)]
% [-cos(theta2)*sin(theta1), cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)), - cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)),      L4*cos(theta1)*sin(theta3) - C51*cos(theta2)*sin(theta1) + C52*cos(theta1)*cos(theta3)*cos(theta4) - C53*cos(theta1)*cos(theta3)*sin(theta4) - C53*cos(theta1)*cos(theta4)*sin(theta3) - C52*cos(theta1)*sin(theta3)*sin(theta4) + L4*cos(theta3)*sin(theta1)*sin(theta2) - C53*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) - C52*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - C52*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) + C53*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)]
% [            -sin(theta2),                                                                                                                          sin(theta3 + theta4)*cos(theta2),                                                                                                                            cos(theta3 + theta4)*cos(theta2),                                                                                                                                                                                                                                                                              C53*cos(theta2)*cos(theta3)*cos(theta4) - C51*sin(theta2) - L4*cos(theta2)*cos(theta3) - L2 + C52*cos(theta2)*cos(theta3)*sin(theta4) + C52*cos(theta2)*cos(theta4)*sin(theta3) - C53*cos(theta2)*sin(theta3)*sin(theta4)]
% [                       0,                                                                                                                                                         0,                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      1]
% 
% 
% T6L =
% 
% [ cos(theta1)*cos(theta2), cos(theta5)*(cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2))) - sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3))), - cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) + sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3))) - sin(theta5)*(cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2))), C61*cos(theta1)*cos(theta2) - L1 + L4*sin(theta1)*sin(theta3) - L4*cos(theta1)*cos(theta3)*sin(theta2) + L5*cos(theta3)*sin(theta1)*sin(theta4) + L5*cos(theta4)*sin(theta1)*sin(theta3) + C62*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1) - L5*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) - C63*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta5) - C63*cos(theta3)*cos(theta5)*sin(theta1)*sin(theta4) - C63*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta3) - C62*cos(theta3)*sin(theta1)*sin(theta4)*sin(theta5) - C62*cos(theta4)*sin(theta1)*sin(theta3)*sin(theta5) - C62*cos(theta5)*sin(theta1)*sin(theta3)*sin(theta4) + L5*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4) + C63*sin(theta1)*sin(theta3)*sin(theta4)*sin(theta5) - C63*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)*sin(theta5) - C63*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3)*sin(theta5) - C63*cos(theta1)*cos(theta5)*sin(theta2)*sin(theta3)*sin(theta4) - C62*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5) + C63*cos(theta1)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2) + C62*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5) + C62*cos(theta1)*cos(theta3)*cos(theta5)*sin(theta2)*sin(theta4) + C62*cos(theta1)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta3)]
% [-cos(theta2)*sin(theta1), cos(theta5)*(cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) - sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) + sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3))), - cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) + sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3))) - sin(theta5)*(cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))),      L4*cos(theta1)*sin(theta3) - C61*cos(theta2)*sin(theta1) + L5*cos(theta1)*cos(theta3)*sin(theta4) + L5*cos(theta1)*cos(theta4)*sin(theta3) + L4*cos(theta3)*sin(theta1)*sin(theta2) + C62*cos(theta1)*cos(theta3)*cos(theta4)*cos(theta5) - C63*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta5) - C63*cos(theta1)*cos(theta3)*cos(theta5)*sin(theta4) - C63*cos(theta1)*cos(theta4)*cos(theta5)*sin(theta3) - C62*cos(theta1)*cos(theta3)*sin(theta4)*sin(theta5) - C62*cos(theta1)*cos(theta4)*sin(theta3)*sin(theta5) - C62*cos(theta1)*cos(theta5)*sin(theta3)*sin(theta4) + L5*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) + C63*cos(theta1)*sin(theta3)*sin(theta4)*sin(theta5) - L5*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4) - C62*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta5) - C62*cos(theta3)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta4) - C62*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3) + C63*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4)*sin(theta5) + C63*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta5) + C63*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4) + C62*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5) - C63*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2)]
% [            -sin(theta2),                                                                                                                                                                                                                                                                                                         sin(theta3 + theta4 + theta5)*cos(theta2),                                                                                                                                                                                                                                                                                                           cos(theta3 + theta4 + theta5)*cos(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 (C63*cos(theta2 + theta3 + theta4 + theta5))/2 - (L5*cos(theta3 - theta2 + theta4))/2 - L2 + (C62*sin(theta2 + theta3 + theta4 + theta5))/2 - (L4*cos(theta2 + theta3))/2 - C61*sin(theta2) + (C63*cos(theta3 - theta2 + theta4 + theta5))/2 + (C62*sin(theta3 - theta2 + theta4 + theta5))/2 - (L4*cos(theta2 - theta3))/2 - (L5*cos(theta2 + theta3 + theta4))/2]
% [                       0,                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  1]
% 
% 
% T2R =
% 
% [ cos(theta6), sin(theta6), 0, L1 - C21*cos(theta6) + C22*sin(theta6)]
% [-sin(theta6), cos(theta6), 0,      C22*cos(theta6) + C21*sin(theta6)]
% [           0,           0, 1,                                    C23]
% [           0,           0, 0,                                      1]
% 
% 
% T3R =
% 
% [ cos(theta6)*cos(theta7), sin(theta6),  cos(theta6)*sin(theta7), L1 + C32*sin(theta6) - L3*sin(theta6) + C31*cos(theta6)*cos(theta7) + C33*cos(theta6)*sin(theta7)]
% [-cos(theta7)*sin(theta6), cos(theta6), -sin(theta6)*sin(theta7),      C32*cos(theta6) - L3*cos(theta6) - C31*cos(theta7)*sin(theta6) - C33*sin(theta6)*sin(theta7)]
% [            -sin(theta7),           0,              cos(theta7),                                                            C33*cos(theta7) - L2 - C31*sin(theta7)]
% [                       0,           0,                        0,                                                                                                 1]
% 
% 
% T4R =
% 
% [ cos(theta6)*cos(theta7), cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8),   cos(theta6)*cos(theta8)*sin(theta7) - sin(theta6)*sin(theta8), L1 + C41*cos(theta6)*cos(theta7) + C42*cos(theta8)*sin(theta6) - C43*sin(theta6)*sin(theta8) + C43*cos(theta6)*cos(theta8)*sin(theta7) + C42*cos(theta6)*sin(theta7)*sin(theta8)]
% [-cos(theta7)*sin(theta6), cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8), - cos(theta6)*sin(theta8) - cos(theta8)*sin(theta6)*sin(theta7),      C42*cos(theta6)*cos(theta8) - C41*cos(theta7)*sin(theta6) - C43*cos(theta6)*sin(theta8) - C43*cos(theta8)*sin(theta6)*sin(theta7) - C42*sin(theta6)*sin(theta7)*sin(theta8)]
% [            -sin(theta7),                                       cos(theta7)*sin(theta8),                                         cos(theta7)*cos(theta8),                                                                                                 C43*cos(theta7)*cos(theta8) - C41*sin(theta7) - L2 + C42*cos(theta7)*sin(theta8)]
% [                       0,                                                             0,                                                               0,                                                                                                                                                                                1]
% 
% 
% T5R =
% 
% [ cos(theta6)*cos(theta7), cos(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7)), - cos(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7)) - sin(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8)), L1 + C51*cos(theta6)*cos(theta7) + L4*sin(theta6)*sin(theta8) + C52*cos(theta8)*cos(theta9)*sin(theta6) - L4*cos(theta6)*cos(theta8)*sin(theta7) - C53*cos(theta8)*sin(theta6)*sin(theta9) - C53*cos(theta9)*sin(theta6)*sin(theta8) - C52*sin(theta6)*sin(theta8)*sin(theta9) + C53*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + C52*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + C52*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8) - C53*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9)]
% [-cos(theta7)*sin(theta6), cos(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7)), - cos(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7)) - sin(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8)),      L4*cos(theta6)*sin(theta8) - C51*cos(theta7)*sin(theta6) + C52*cos(theta6)*cos(theta8)*cos(theta9) - C53*cos(theta6)*cos(theta8)*sin(theta9) - C53*cos(theta6)*cos(theta9)*sin(theta8) - C52*cos(theta6)*sin(theta8)*sin(theta9) + L4*cos(theta8)*sin(theta6)*sin(theta7) - C53*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) - C52*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9) - C52*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8) + C53*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9)]
% [            -sin(theta7),                                                                                                                          sin(theta8 + theta9)*cos(theta7),                                                                                                                            cos(theta8 + theta9)*cos(theta7),                                                                                                                                                                                                                                                                              C53*cos(theta7)*cos(theta8)*cos(theta9) - C51*sin(theta7) - L4*cos(theta7)*cos(theta8) - L2 + C52*cos(theta7)*cos(theta8)*sin(theta9) + C52*cos(theta7)*cos(theta9)*sin(theta8) - C53*cos(theta7)*sin(theta8)*sin(theta9)]
% [                       0,                                                                                                                                                         0,                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      1]
% 
% 
% T6R =
% 
% [ cos(theta6)*cos(theta7), cos(theta10)*(cos(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7))) - sin(theta10)*(cos(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7)) + sin(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8))), - cos(theta10)*(cos(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7)) + sin(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8))) - sin(theta10)*(cos(theta9)*(cos(theta8)*sin(theta6) + cos(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(sin(theta6)*sin(theta8) - cos(theta6)*cos(theta8)*sin(theta7))), L1 + C61*cos(theta6)*cos(theta7) + L4*sin(theta6)*sin(theta8) - L4*cos(theta6)*cos(theta8)*sin(theta7) + L5*cos(theta8)*sin(theta6)*sin(theta9) + L5*cos(theta9)*sin(theta6)*sin(theta8) + C62*cos(theta8)*cos(theta9)*cos(theta10)*sin(theta6) - L5*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) - C63*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta10) - C63*cos(theta8)*cos(theta10)*sin(theta6)*sin(theta9) - C63*cos(theta9)*cos(theta10)*sin(theta6)*sin(theta8) - C62*cos(theta8)*sin(theta6)*sin(theta9)*sin(theta10) - C62*cos(theta9)*sin(theta6)*sin(theta8)*sin(theta10) - C62*cos(theta10)*sin(theta6)*sin(theta8)*sin(theta9) + L5*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9) + C63*sin(theta6)*sin(theta8)*sin(theta9)*sin(theta10) - C63*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9)*sin(theta10) - C63*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8)*sin(theta10) - C63*cos(theta6)*cos(theta10)*sin(theta7)*sin(theta8)*sin(theta9) - C62*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9)*sin(theta10) + C63*cos(theta6)*cos(theta8)*cos(theta9)*cos(theta10)*sin(theta7) + C62*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7)*sin(theta10) + C62*cos(theta6)*cos(theta8)*cos(theta10)*sin(theta7)*sin(theta9) + C62*cos(theta6)*cos(theta9)*cos(theta10)*sin(theta7)*sin(theta8)]
% [-cos(theta7)*sin(theta6), cos(theta10)*(cos(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7))) - sin(theta10)*(cos(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7)) + sin(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8))), - cos(theta10)*(cos(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7)) + sin(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8))) - sin(theta10)*(cos(theta9)*(cos(theta6)*cos(theta8) - sin(theta6)*sin(theta7)*sin(theta8)) - sin(theta9)*(cos(theta6)*sin(theta8) + cos(theta8)*sin(theta6)*sin(theta7))),      L4*cos(theta6)*sin(theta8) - C61*cos(theta7)*sin(theta6) + L5*cos(theta6)*cos(theta8)*sin(theta9) + L5*cos(theta6)*cos(theta9)*sin(theta8) + L4*cos(theta8)*sin(theta6)*sin(theta7) + C62*cos(theta6)*cos(theta8)*cos(theta9)*cos(theta10) - C63*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta10) - C63*cos(theta6)*cos(theta8)*cos(theta10)*sin(theta9) - C63*cos(theta6)*cos(theta9)*cos(theta10)*sin(theta8) - C62*cos(theta6)*cos(theta8)*sin(theta9)*sin(theta10) - C62*cos(theta6)*cos(theta9)*sin(theta8)*sin(theta10) - C62*cos(theta6)*cos(theta10)*sin(theta8)*sin(theta9) + L5*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) + C63*cos(theta6)*sin(theta8)*sin(theta9)*sin(theta10) - L5*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9) - C62*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta10) - C62*cos(theta8)*cos(theta10)*sin(theta6)*sin(theta7)*sin(theta9) - C62*cos(theta9)*cos(theta10)*sin(theta6)*sin(theta7)*sin(theta8) + C63*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9)*sin(theta10) + C63*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta10) + C63*cos(theta10)*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9) + C62*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9)*sin(theta10) - C63*cos(theta8)*cos(theta9)*cos(theta10)*sin(theta6)*sin(theta7)]
% [            -sin(theta7),                                                                                                                                                                                                                                                                                                          sin(theta8 + theta9 + theta10)*cos(theta7),                                                                                                                                                                                                                                                                                                            cos(theta8 + theta9 + theta10)*cos(theta7),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             (C63*cos(theta7 + theta8 + theta9 + theta10))/2 - (L5*cos(theta8 - theta7 + theta9))/2 - L2 + (C62*sin(theta7 + theta8 + theta9 + theta10))/2 - (L4*cos(theta7 + theta8))/2 - C61*sin(theta7) + (C63*cos(theta8 - theta7 + theta9 + theta10))/2 + (C62*sin(theta8 - theta7 + theta9 + theta10))/2 - (L4*cos(theta7 - theta8))/2 - (L5*cos(theta7 + theta8 + theta9))/2]
% [                       0,                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  1]
