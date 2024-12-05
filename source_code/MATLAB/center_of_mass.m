%% for V2 robot
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

% substitute values
val_theta1 = 0;
val_theta2 = 0;
val_theta3 = 0;
val_theta4 = 0;
val_theta6 = 0;
val_theta7 = 0;
val_theta8 = 0;
val_theta9 = 0;
val_L1 = 0.135;
val_L2 = 0.12;
val_L3 = 0.09;
val_L4 = 0.18;
val_L5 = 0.27;
m1 = 1.73;
m2 = 0.55;
m3 = 0.38;
m4 = 0.53;
m5 = 0.42;

% link's relative offset from the last joint
syms C11 C12 C13 C21 C22 C23 C31 C32 C33 C41 C42 C43 C51 C52 C53
C1 = [C11 C12 C13];
C2 = [C21 C22 C23];
C3 = [C31 C32 C33];
C4 = [C41 C42 C43];
C5 = [C51 C52 C53];
val_C1 = [0 0 0.027];
val_C2 = [0.0075 -0.06 -0.067];
val_C3 = [0 0.084 0];
val_C4 = [0 0 -0.142];
val_C5 = [0 0 -0.131];

sub_vals = [val_theta1 val_theta2 val_theta3 val_theta4 val_theta6 val_theta7...
    val_theta8 val_theta9 val_L1 val_L2 val_L3 val_L4 val_L5 val_C1 val_C2 val_C3 val_C4 val_C5];
sub_syms = [theta1 theta2 theta3 theta4 theta6 theta7 theta8 theta9 L1 L2 L3 L4 L5 C1 C2 C3 C4 C5];

M1 = [1 0 0 C1(1);
    0 1 0 C1(2);
    0 0 1 C1(3);
    0 0 0 1];
M2 = [1 0 0 L1+C2(1);
    0 1 0 C2(2);
    0 0 1 C2(3);
    0 0 0 1];
M3 = [1 0 0 L1+C3(1);
    0 1 0 -L3+C3(2);
    0 0 1 -L2+C3(3);
    0 0 0 1];
M4 = [1 0 0 L1+C4(1);
    0 1 0 C4(2);
    0 0 1 -L2+C4(3);
    0 0 0 1];
M5 = [1 0 0 L1+C5(1);
    0 1 0 C5(2);
    0 0 1 -(L2+L4)+C5(3);
    0 0 0 1];
M6 = [1 0 0 -L1+C2(1);
    0 1 0 C2(2);
    0 0 1 C2(3);
    0 0 0 1];
M7 = [1 0 0 -L1+C3(1);
    0 1 0 -L3+C3(2);
    0 0 1 -L2+C3(3);
    0 0 0 1];
M8 = [1 0 0 -L1+C4(1);
    0 1 0 C4(2);
    0 0 1 -L2+C4(3);
    0 0 0 1];
M9 = [1 0 0 -L1+C5(1);
    0 1 0 C5(2);
    0 0 1 -(L2+L4)+C5(3);
    0 0 0 1];
T1 = M1
T2 = simplify(expm(S{1}*theta1)*M2)
T3 = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*M3)
T4 = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*M4)
T5 = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*expm(S{4}*theta4)*M5)

T6 = simplify(expm(S{6}*theta1)*M6)
T7 = simplify(expm(S{6}*theta1)*expm(S{7}*theta2)*M7)
T8 = simplify(expm(S{6}*theta1)*expm(S{7}*theta2)*expm(S{8}*theta3)*M8)
T9 = simplify(expm(S{6}*theta1)*expm(S{7}*theta2)*expm(S{8}*theta3)*expm(S{9}*theta4)*M9)

T = {T1,T2,T3,T4,T5,T6,T7,T8,T9};
m = {m1,m2,m3,m4,m5,m2,m3,m4,m5};
% CoM calculation
x_total = 0;
y_total = 0;
z_total = 0;
m_total = 0;
for i = 1:9
    T{i} = vpa(subs(T{i},sub_syms,sub_vals),2);
    x_total = x_total + T{i}(1,4)*m{i};
    y_total = y_total + T{i}(2,4)*m{i};
    z_total = z_total + T{i}(3,4)*m{i};
    m_total = m_total + m{i};
end
x_total = x_total/m_total;
y_total = y_total/m_total;
z_total = z_total/m_total;


% The transformation matrixes are for Orin computation
% T1 =
% 
% [1, 0, 0, C11]
% [0, 1, 0, C12]
% [0, 0, 1, C13]
% [0, 0, 0,   1]
% 
% 
% T2 =
% 
% [ cos(theta1), sin(theta1), 0, C21*cos(theta1) - L1 + 2*L1*cos(theta1) + C22*sin(theta1)]
% [-sin(theta1), cos(theta1), 0,      C22*cos(theta1) - C21*sin(theta1) - 2*L1*sin(theta1)]
% [           0,           0, 1,                                                       C23]
% [           0,           0, 0,                                                         1]
% 
% 
% T3 =
% 
% [ cos(theta1)*cos(theta2), sin(theta1),  cos(theta1)*sin(theta2), C32*sin(theta1) - L1 - L3*sin(theta1) + C31*cos(theta1)*cos(theta2) + 2*L1*cos(theta1)*cos(theta2) + C33*cos(theta1)*sin(theta2)]
% [-cos(theta2)*sin(theta1), cos(theta1), -sin(theta1)*sin(theta2),      C32*cos(theta1) - L3*cos(theta1) - C31*cos(theta2)*sin(theta1) - 2*L1*cos(theta2)*sin(theta1) - C33*sin(theta1)*sin(theta2)]
% [            -sin(theta2),           0,              cos(theta2),                                                                        C33*cos(theta2) - L2 - C31*sin(theta2) - 2*L1*sin(theta2)]
% [                       0,           0,                        0,                                                                                                                                1]
% 
% 
% T4 =
% 
% [ cos(theta1)*cos(theta2), cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3),   cos(theta1)*cos(theta3)*sin(theta2) - sin(theta1)*sin(theta3), C41*cos(theta1)*cos(theta2) - L1 + 2*L1*cos(theta1)*cos(theta2) + C42*cos(theta3)*sin(theta1) - C43*sin(theta1)*sin(theta3) + C43*cos(theta1)*cos(theta3)*sin(theta2) + C42*cos(theta1)*sin(theta2)*sin(theta3)]
% [-cos(theta2)*sin(theta1), cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3), - cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2),      C42*cos(theta1)*cos(theta3) - C41*cos(theta2)*sin(theta1) - C43*cos(theta1)*sin(theta3) - 2*L1*cos(theta2)*sin(theta1) - C43*cos(theta3)*sin(theta1)*sin(theta2) - C42*sin(theta1)*sin(theta2)*sin(theta3)]
% [            -sin(theta2),                                       cos(theta2)*sin(theta3),                                         cos(theta2)*cos(theta3),                                                                                                             C43*cos(theta2)*cos(theta3) - C41*sin(theta2) - 2*L1*sin(theta2) - L2 + C42*cos(theta2)*sin(theta3)]
% [                       0,                                                             0,                                                               0,                                                                                                                                                                                                               1]
% 
% 
% T5 =
% 
% [ cos(theta1)*cos(theta2), cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)), - cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)), C51*cos(theta1)*cos(theta2) - L1 + 2*L1*cos(theta1)*cos(theta2) + L4*sin(theta1)*sin(theta3) + C52*cos(theta3)*cos(theta4)*sin(theta1) - L4*cos(theta1)*cos(theta3)*sin(theta2) - C53*cos(theta3)*sin(theta1)*sin(theta4) - C53*cos(theta4)*sin(theta1)*sin(theta3) - C52*sin(theta1)*sin(theta3)*sin(theta4) + C53*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C52*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C52*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) - C53*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)]
% [-cos(theta2)*sin(theta1), cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)), - cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)),      L4*cos(theta1)*sin(theta3) - 2*L1*cos(theta2)*sin(theta1) - C51*cos(theta2)*sin(theta1) + C52*cos(theta1)*cos(theta3)*cos(theta4) - C53*cos(theta1)*cos(theta3)*sin(theta4) - C53*cos(theta1)*cos(theta4)*sin(theta3) - C52*cos(theta1)*sin(theta3)*sin(theta4) + L4*cos(theta3)*sin(theta1)*sin(theta2) - C53*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) - C52*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - C52*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) + C53*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)]
% [            -sin(theta2),                                                                                                                          sin(theta3 + theta4)*cos(theta2),                                                                                                                            cos(theta3 + theta4)*cos(theta2),                                                                                                                                                                                                                                                                                          C53*cos(theta2)*cos(theta3)*cos(theta4) - C51*sin(theta2) - 2*L1*sin(theta2) - L4*cos(theta2)*cos(theta3) - L2 + C52*cos(theta2)*cos(theta3)*sin(theta4) + C52*cos(theta2)*cos(theta4)*sin(theta3) - C53*cos(theta2)*sin(theta3)*sin(theta4)]
% [                       0,                                                                                                                                                         0,                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     1]
% 
% 
% T6 =
% 
% [ cos(theta1), sin(theta1), 0, L1 + C21*cos(theta1) - 2*L1*cos(theta1) + C22*sin(theta1)]
% [-sin(theta1), cos(theta1), 0,      C22*cos(theta1) - C21*sin(theta1) + 2*L1*sin(theta1)]
% [           0,           0, 1,                                                       C23]
% [           0,           0, 0,                                                         1]
% 
% 
% T7 =
% 
% [ cos(theta1)*cos(theta2), sin(theta1),  cos(theta1)*sin(theta2), L1 + C32*sin(theta1) - L3*sin(theta1) + C31*cos(theta1)*cos(theta2) - 2*L1*cos(theta1)*cos(theta2) + C33*cos(theta1)*sin(theta2)]
% [-cos(theta2)*sin(theta1), cos(theta1), -sin(theta1)*sin(theta2),      C32*cos(theta1) - L3*cos(theta1) - C31*cos(theta2)*sin(theta1) + 2*L1*cos(theta2)*sin(theta1) - C33*sin(theta1)*sin(theta2)]
% [            -sin(theta2),           0,              cos(theta2),                                                                        C33*cos(theta2) - L2 - C31*sin(theta2) + 2*L1*sin(theta2)]
% [                       0,           0,                        0,                                                                                                                                1]
% 
% 
% T8 =
% 
% [ cos(theta1)*cos(theta2), cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3),   cos(theta1)*cos(theta3)*sin(theta2) - sin(theta1)*sin(theta3), L1 + C41*cos(theta1)*cos(theta2) - 2*L1*cos(theta1)*cos(theta2) + C42*cos(theta3)*sin(theta1) - C43*sin(theta1)*sin(theta3) + C43*cos(theta1)*cos(theta3)*sin(theta2) + C42*cos(theta1)*sin(theta2)*sin(theta3)]
% [-cos(theta2)*sin(theta1), cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3), - cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2),      C42*cos(theta1)*cos(theta3) - C41*cos(theta2)*sin(theta1) - C43*cos(theta1)*sin(theta3) + 2*L1*cos(theta2)*sin(theta1) - C43*cos(theta3)*sin(theta1)*sin(theta2) - C42*sin(theta1)*sin(theta2)*sin(theta3)]
% [            -sin(theta2),                                       cos(theta2)*sin(theta3),                                         cos(theta2)*cos(theta3),                                                                                                             2*L1*sin(theta2) - C41*sin(theta2) - L2 + C43*cos(theta2)*cos(theta3) + C42*cos(theta2)*sin(theta3)]
% [                       0,                                                             0,                                                               0,                                                                                                                                                                                                               1]
% 
% 
% T9 =
% 
% [ cos(theta1)*cos(theta2), cos(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)), - cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) - sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)), L1 + C51*cos(theta1)*cos(theta2) - 2*L1*cos(theta1)*cos(theta2) + L4*sin(theta1)*sin(theta3) + C52*cos(theta3)*cos(theta4)*sin(theta1) - L4*cos(theta1)*cos(theta3)*sin(theta2) - C53*cos(theta3)*sin(theta1)*sin(theta4) - C53*cos(theta4)*sin(theta1)*sin(theta3) - C52*sin(theta1)*sin(theta3)*sin(theta4) + C53*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C52*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C52*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) - C53*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)]
% [-cos(theta2)*sin(theta1), cos(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)), - cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - sin(theta4)*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)),      2*L1*cos(theta2)*sin(theta1) - C51*cos(theta2)*sin(theta1) + L4*cos(theta1)*sin(theta3) + C52*cos(theta1)*cos(theta3)*cos(theta4) - C53*cos(theta1)*cos(theta3)*sin(theta4) - C53*cos(theta1)*cos(theta4)*sin(theta3) - C52*cos(theta1)*sin(theta3)*sin(theta4) + L4*cos(theta3)*sin(theta1)*sin(theta2) - C53*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) - C52*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - C52*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) + C53*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)]
% [            -sin(theta2),                                                                                                                          sin(theta3 + theta4)*cos(theta2),                                                                                                                            cos(theta3 + theta4)*cos(theta2),                                                                                                                                                                                                                                                                                          2*L1*sin(theta2) - C51*sin(theta2) - L2 - L4*cos(theta2)*cos(theta3) + C53*cos(theta2)*cos(theta3)*cos(theta4) + C52*cos(theta2)*cos(theta3)*sin(theta4) + C52*cos(theta2)*cos(theta4)*sin(theta3) - C53*cos(theta2)*sin(theta3)*sin(theta4)]
% [                       0,                                                                                                                                                         0,                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     1]
%% for V1 robot
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
T_01 = sym(eye(4));
T_02 = sym(eye(4));
T_03 = sym(eye(4));
T_04 = sym(eye(4));
T_05 = sym(eye(4));
for i = 1:1
    T(:,:,i) = [cos(dhparams (i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_01 = T_01*T(:,:,i);
end
for i = 1:2
    T(:,:,i) = [cos(dhparams (i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_02 = T_02*T(:,:,i);
end
for i = 1:3
    T(:,:,i) = [cos(dhparams (i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_03 = T_03*T(:,:,i);
end
for i = 1:4
    T(:,:,i) = [cos(dhparams (i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_04 = T_04*T(:,:,i);
end
for i = 1:5
    T(:,:,i) = [cos(dhparams (i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_05 = T_05*T(:,:,i);
end
