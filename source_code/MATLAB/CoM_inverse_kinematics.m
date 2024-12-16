%% for V2 robot
clear; clc; close all;
syms theta1 theta2 theta3 theta4 theta6 theta7 theta8 theta9 L1 L2 L3 L4 L5 m1 m2 m3 m4 m5 real

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
val_theta1 = 0.0001;
val_theta2 = 0.0001;
val_theta3 = 0.0001;
val_theta4 = 0.0001;
val_theta6 = 0.0001;
val_theta7 = 0.0001;
val_theta8 = 0.0001;
val_theta9 = 0.0001;
val_L1 = 0.135;
val_L2 = 0.12;
val_L3 = 0.09;
val_L4 = 0.18;
val_L5 = 0.27;
val_m1 = 1.73;
val_m2 = 0.55;
val_m3 = 0.38;
val_m4 = 0.53;
val_m5 = 0.42;

% link's relative offset from the last joint
syms C11 C12 C13 C21 C22 C23 C31 C32 C33 C41 C42 C43 C51 C52 C53
C1 = [C11 C12 C13];
C2 = [C21 C22 C23];
C3 = [C31 C32 C33];
C4 = [C41 C42 C43];
C5 = [C51 C52 C53];
val_C1 = [0 0 0.027];
val_C2 = [-0.0075 -0.06 -0.067];
val_C3 = [0 0.084 0];
val_C4 = [0 0 -0.142];
val_C5 = [0 0 -0.131];

sub_vals = [val_theta1 val_theta6 val_L1 val_L2 val_L3 val_L4 val_L5 val_C1 val_C2 val_C3 val_C4 val_C5 val_m1 val_m2 val_m3 val_m4 val_m5];
sub_syms = [theta1 theta6 L1 L2 L3 L4 L5 C1 C2 C3 C4 C5 m1 m2 m3 m4 m5];

M1 = [1 0 0 C1(1);
    0 1 0 C1(2);
    0 0 1 C1(3);
    0 0 0 1];
M2 = [1 0 0 -L1+C2(1);
    0 1 0 C2(2);
    0 0 1 C2(3);
    0 0 0 1];
M3 = [1 0 0 -L1+C3(1);
    0 1 0 -L3+C3(2);
    0 0 1 -L2+C3(3);
    0 0 0 1];
M4 = [1 0 0 -L1+C4(1);
    0 1 0 C4(2);
    0 0 1 -L2+C4(3);
    0 0 0 1];
M5 = [1 0 0 -L1+C5(1);
    0 1 0 C5(2);
    0 0 1 -(L2+L4)+C5(3);
    0 0 0 1];
M6 = [1 0 0 L1-C2(1);
    0 1 0 C2(2);
    0 0 1 C2(3);
    0 0 0 1];
M7 = [1 0 0 L1+C3(1);
    0 1 0 -L3+C3(2);
    0 0 1 -L2+C3(3);
    0 0 0 1];
M8 = [1 0 0 L1+C4(1);
    0 1 0 C4(2);
    0 0 1 -L2+C4(3);
    0 0 0 1];
M9 = [1 0 0 L1+C5(1);
    0 1 0 C5(2);
    0 0 1 -(L2+L4)+C5(3);
    0 0 0 1];
T1 = M1;
T2 = simplify(expm(S{1}*theta1)*M2);
T3 = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*M3);
T4 = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*M4);
T5 = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*expm(S{4}*theta4)*M5);

T6 = simplify(expm(S{6}*theta6)*M6);
T7 = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*M7);
T8 = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*expm(S{8}*theta8)*M8);
T9 = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*expm(S{8}*theta8)*expm(S{9}*theta9)*M9);

T = {T1,T2,T3,T4,T5,T6,T7,T8,T9};
m = {m1,m2,m3,m4,m5,m2,m3,m4,m5};
% CoM calculation
x_total = 0;
y_total = 0;
z_total = 0;
m_total = 0;
for i = 1:9
    x_total = x_total + T{i}(1,4)*m{i};
    y_total = y_total + T{i}(2,4)*m{i};
    z_total = z_total + T{i}(3,4)*m{i};
    m_total = m_total + m{i};
end
x_total = x_total/m_total;
y_total = y_total/m_total;
z_total = z_total/m_total;

R = sym('R%d%d',[3 3]);
sym_X_act = [x_total;y_total;z_total];
sym_X_act = R*sym_X_act;
X_ref = [0;0;-0.135];
sym_theta = [theta2 theta3 theta4 theta7 theta8 theta9];
val_theta = [val_theta2 val_theta3 val_theta4 val_theta7 val_theta8 val_theta9];
jacob = simplify(jacobian(sym_X_act,sym_theta));
jacob11 = jacob(1,1)
jacob12 = jacob(1,2)
jacob13 = jacob(1,3)
jacob14 = jacob(1,4)
jacob15 = jacob(1,5)
jacob16 = jacob(1,6)
jacob21 = jacob(2,1)
jacob22 = jacob(2,2)
jacob23 = jacob(2,3)
jacob24 = jacob(2,4)
jacob25 = jacob(2,5)
jacob26 = jacob(2,6)
jacob31 = jacob(3,1)
jacob32 = jacob(3,2)
jacob33 = jacob(3,3)
jacob34 = jacob(3,4)
jacob35 = jacob(3,5)
jacob36 = jacob(3,6)
% norm_e = 1;
% epsilon = 0.005;
% iteration = 0;
% 
% while norm_e > epsilon && iteration < 100
%     val_X_act = vpa(subs(sym_X_act,[sym_theta sub_syms],[val_theta sub_vals]),8);
%     error_X = vpa(X_ref - val_X_act,8);
%     norm_e = norm(error_X);
%     val_jacob = vpa(subs(jacob,[sym_theta sub_syms],[val_theta sub_vals]),8);
%     inv_jacob = pinv(val_jacob);
%     delta_theta = inv_jacob*error_X;
%     val_theta = val_theta + delta_theta';
%     val_theta = mod(eval(val_theta) + pi, 2*pi) - pi;
%     iteration = iteration + 1;
% end
% val_theta
% val_X_act

% The jacobian matrix are for Orin computation
% jacob11 =
% (R11*cos(theta1)*(C33*m3*cos(theta2) - C31*m3*sin(theta2) - C41*m4*sin(theta2) - C51*m5*sin(theta2) + C43*m4*cos(theta2)*cos(theta3) - L4*m5*cos(theta2)*cos(theta3) + C42*m4*cos(theta2)*sin(theta3) + C53*m5*cos(theta2)*cos(theta3)*cos(theta4) + C52*m5*cos(theta2)*cos(theta3)*sin(theta4) + C52*m5*cos(theta2)*cos(theta4)*sin(theta3) - C53*m5*cos(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R13*(C31*m3*cos(theta2) + C41*m4*cos(theta2) + C51*m5*cos(theta2) + C33*m3*sin(theta2) + C43*m4*cos(theta3)*sin(theta2) - L4*m5*cos(theta3)*sin(theta2) + C42*m4*sin(theta2)*sin(theta3) + C53*m5*cos(theta3)*cos(theta4)*sin(theta2) + C52*m5*cos(theta3)*sin(theta2)*sin(theta4) + C52*m5*cos(theta4)*sin(theta2)*sin(theta3) - C53*m5*sin(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R12*sin(theta1)*(C33*m3*cos(theta2) - C31*m3*sin(theta2) - C41*m4*sin(theta2) - C51*m5*sin(theta2) + C43*m4*cos(theta2)*cos(theta3) - L4*m5*cos(theta2)*cos(theta3) + C42*m4*cos(theta2)*sin(theta3) + C53*m5*cos(theta2)*cos(theta3)*cos(theta4) + C52*m5*cos(theta2)*cos(theta3)*sin(theta4) + C52*m5*cos(theta2)*cos(theta4)*sin(theta3) - C53*m5*cos(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob12 =
% (R13*cos(theta2)*(L4*m5*sin(theta3) + C52*m5*cos(theta3 + theta4) - C53*m5*sin(theta3 + theta4) + C42*m4*cos(theta3) - C43*m4*sin(theta3)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R12*(m4*(C43*cos(theta1)*cos(theta3) + C42*cos(theta1)*sin(theta3) + C42*cos(theta3)*sin(theta1)*sin(theta2) - C43*sin(theta1)*sin(theta2)*sin(theta3)) - m5*(L4*cos(theta1)*cos(theta3) - C53*cos(theta1)*cos(theta3)*cos(theta4) - C52*cos(theta1)*cos(theta3)*sin(theta4) - C52*cos(theta1)*cos(theta4)*sin(theta3) + C53*cos(theta1)*sin(theta3)*sin(theta4) - L4*sin(theta1)*sin(theta2)*sin(theta3) - C52*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) + C53*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) + C53*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) + C52*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R11*(m4*(C43*cos(theta3)*sin(theta1) + C42*sin(theta1)*sin(theta3) - C42*cos(theta1)*cos(theta3)*sin(theta2) + C43*cos(theta1)*sin(theta2)*sin(theta3)) + m5*(C53*cos(theta3)*cos(theta4)*sin(theta1) - L4*cos(theta3)*sin(theta1) + C52*cos(theta3)*sin(theta1)*sin(theta4) + C52*cos(theta4)*sin(theta1)*sin(theta3) - L4*cos(theta1)*sin(theta2)*sin(theta3) - C53*sin(theta1)*sin(theta3)*sin(theta4) - C52*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C53*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C53*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) + C52*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob13 =
% -(m5*(C53*R13*sin(theta3 + theta4)*cos(theta2) - C52*R13*cos(theta3 + theta4)*cos(theta2) + C53*R12*cos(theta1)*cos(theta3)*cos(theta4) + C52*R12*cos(theta1)*cos(theta3)*sin(theta4) + C52*R12*cos(theta1)*cos(theta4)*sin(theta3) + C53*R11*cos(theta3)*cos(theta4)*sin(theta1) + C52*R11*cos(theta3)*sin(theta1)*sin(theta4) + C52*R11*cos(theta4)*sin(theta1)*sin(theta3) - C53*R12*cos(theta1)*sin(theta3)*sin(theta4) - C53*R11*sin(theta1)*sin(theta3)*sin(theta4) - C52*R11*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C52*R12*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) + C53*R11*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C53*R11*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) + C52*R11*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4) - C53*R12*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - C53*R12*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) - C52*R12*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob14 =
% (R11*cos(theta6)*(C33*m3*cos(theta7) - C31*m3*sin(theta7) - C41*m4*sin(theta7) - C51*m5*sin(theta7) + C43*m4*cos(theta7)*cos(theta8) - L4*m5*cos(theta7)*cos(theta8) + C42*m4*cos(theta7)*sin(theta8) + C53*m5*cos(theta7)*cos(theta8)*cos(theta9) + C52*m5*cos(theta7)*cos(theta8)*sin(theta9) + C52*m5*cos(theta7)*cos(theta9)*sin(theta8) - C53*m5*cos(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R13*(C31*m3*cos(theta7) + C41*m4*cos(theta7) + C51*m5*cos(theta7) + C33*m3*sin(theta7) + C43*m4*cos(theta8)*sin(theta7) - L4*m5*cos(theta8)*sin(theta7) + C42*m4*sin(theta7)*sin(theta8) + C53*m5*cos(theta8)*cos(theta9)*sin(theta7) + C52*m5*cos(theta8)*sin(theta7)*sin(theta9) + C52*m5*cos(theta9)*sin(theta7)*sin(theta8) - C53*m5*sin(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R12*sin(theta6)*(C33*m3*cos(theta7) - C31*m3*sin(theta7) - C41*m4*sin(theta7) - C51*m5*sin(theta7) + C43*m4*cos(theta7)*cos(theta8) - L4*m5*cos(theta7)*cos(theta8) + C42*m4*cos(theta7)*sin(theta8) + C53*m5*cos(theta7)*cos(theta8)*cos(theta9) + C52*m5*cos(theta7)*cos(theta8)*sin(theta9) + C52*m5*cos(theta7)*cos(theta9)*sin(theta8) - C53*m5*cos(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob15 =
% (R13*cos(theta7)*(L4*m5*sin(theta8) + C52*m5*cos(theta8 + theta9) - C53*m5*sin(theta8 + theta9) + C42*m4*cos(theta8) - C43*m4*sin(theta8)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R12*(m4*(C43*cos(theta6)*cos(theta8) + C42*cos(theta6)*sin(theta8) + C42*cos(theta8)*sin(theta6)*sin(theta7) - C43*sin(theta6)*sin(theta7)*sin(theta8)) - m5*(L4*cos(theta6)*cos(theta8) - C53*cos(theta6)*cos(theta8)*cos(theta9) - C52*cos(theta6)*cos(theta8)*sin(theta9) - C52*cos(theta6)*cos(theta9)*sin(theta8) + C53*cos(theta6)*sin(theta8)*sin(theta9) - L4*sin(theta6)*sin(theta7)*sin(theta8) - C52*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) + C53*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9) + C53*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8) + C52*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R11*(m4*(C43*cos(theta8)*sin(theta6) + C42*sin(theta6)*sin(theta8) - C42*cos(theta6)*cos(theta8)*sin(theta7) + C43*cos(theta6)*sin(theta7)*sin(theta8)) + m5*(C53*cos(theta8)*cos(theta9)*sin(theta6) - L4*cos(theta8)*sin(theta6) + C52*cos(theta8)*sin(theta6)*sin(theta9) + C52*cos(theta9)*sin(theta6)*sin(theta8) - L4*cos(theta6)*sin(theta7)*sin(theta8) - C53*sin(theta6)*sin(theta8)*sin(theta9) - C52*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + C53*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + C53*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8) + C52*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob16 =
% -(m5*(C53*R13*sin(theta8 + theta9)*cos(theta7) - C52*R13*cos(theta8 + theta9)*cos(theta7) + C53*R12*cos(theta6)*cos(theta8)*cos(theta9) + C52*R12*cos(theta6)*cos(theta8)*sin(theta9) + C52*R12*cos(theta6)*cos(theta9)*sin(theta8) + C53*R11*cos(theta8)*cos(theta9)*sin(theta6) + C52*R11*cos(theta8)*sin(theta6)*sin(theta9) + C52*R11*cos(theta9)*sin(theta6)*sin(theta8) - C53*R12*cos(theta6)*sin(theta8)*sin(theta9) - C53*R11*sin(theta6)*sin(theta8)*sin(theta9) - C52*R11*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + C52*R12*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) + C53*R11*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + C53*R11*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8) + C52*R11*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9) - C53*R12*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9) - C53*R12*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8) - C52*R12*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob21 =
% (R21*cos(theta1)*(C33*m3*cos(theta2) - C31*m3*sin(theta2) - C41*m4*sin(theta2) - C51*m5*sin(theta2) + C43*m4*cos(theta2)*cos(theta3) - L4*m5*cos(theta2)*cos(theta3) + C42*m4*cos(theta2)*sin(theta3) + C53*m5*cos(theta2)*cos(theta3)*cos(theta4) + C52*m5*cos(theta2)*cos(theta3)*sin(theta4) + C52*m5*cos(theta2)*cos(theta4)*sin(theta3) - C53*m5*cos(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R23*(C31*m3*cos(theta2) + C41*m4*cos(theta2) + C51*m5*cos(theta2) + C33*m3*sin(theta2) + C43*m4*cos(theta3)*sin(theta2) - L4*m5*cos(theta3)*sin(theta2) + C42*m4*sin(theta2)*sin(theta3) + C53*m5*cos(theta3)*cos(theta4)*sin(theta2) + C52*m5*cos(theta3)*sin(theta2)*sin(theta4) + C52*m5*cos(theta4)*sin(theta2)*sin(theta3) - C53*m5*sin(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R22*sin(theta1)*(C33*m3*cos(theta2) - C31*m3*sin(theta2) - C41*m4*sin(theta2) - C51*m5*sin(theta2) + C43*m4*cos(theta2)*cos(theta3) - L4*m5*cos(theta2)*cos(theta3) + C42*m4*cos(theta2)*sin(theta3) + C53*m5*cos(theta2)*cos(theta3)*cos(theta4) + C52*m5*cos(theta2)*cos(theta3)*sin(theta4) + C52*m5*cos(theta2)*cos(theta4)*sin(theta3) - C53*m5*cos(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob22 =
% (R23*cos(theta2)*(L4*m5*sin(theta3) + C52*m5*cos(theta3 + theta4) - C53*m5*sin(theta3 + theta4) + C42*m4*cos(theta3) - C43*m4*sin(theta3)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R22*(m4*(C43*cos(theta1)*cos(theta3) + C42*cos(theta1)*sin(theta3) + C42*cos(theta3)*sin(theta1)*sin(theta2) - C43*sin(theta1)*sin(theta2)*sin(theta3)) - m5*(L4*cos(theta1)*cos(theta3) - C53*cos(theta1)*cos(theta3)*cos(theta4) - C52*cos(theta1)*cos(theta3)*sin(theta4) - C52*cos(theta1)*cos(theta4)*sin(theta3) + C53*cos(theta1)*sin(theta3)*sin(theta4) - L4*sin(theta1)*sin(theta2)*sin(theta3) - C52*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) + C53*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) + C53*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) + C52*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R21*(m4*(C43*cos(theta3)*sin(theta1) + C42*sin(theta1)*sin(theta3) - C42*cos(theta1)*cos(theta3)*sin(theta2) + C43*cos(theta1)*sin(theta2)*sin(theta3)) + m5*(C53*cos(theta3)*cos(theta4)*sin(theta1) - L4*cos(theta3)*sin(theta1) + C52*cos(theta3)*sin(theta1)*sin(theta4) + C52*cos(theta4)*sin(theta1)*sin(theta3) - L4*cos(theta1)*sin(theta2)*sin(theta3) - C53*sin(theta1)*sin(theta3)*sin(theta4) - C52*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C53*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C53*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) + C52*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob23 =
% -(m5*(C53*R23*sin(theta3 + theta4)*cos(theta2) - C52*R23*cos(theta3 + theta4)*cos(theta2) + C53*R22*cos(theta1)*cos(theta3)*cos(theta4) + C52*R22*cos(theta1)*cos(theta3)*sin(theta4) + C52*R22*cos(theta1)*cos(theta4)*sin(theta3) + C53*R21*cos(theta3)*cos(theta4)*sin(theta1) + C52*R21*cos(theta3)*sin(theta1)*sin(theta4) + C52*R21*cos(theta4)*sin(theta1)*sin(theta3) - C53*R22*cos(theta1)*sin(theta3)*sin(theta4) - C53*R21*sin(theta1)*sin(theta3)*sin(theta4) - C52*R21*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C52*R22*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) + C53*R21*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C53*R21*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) + C52*R21*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4) - C53*R22*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - C53*R22*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) - C52*R22*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob24 =
% (R21*cos(theta6)*(C33*m3*cos(theta7) - C31*m3*sin(theta7) - C41*m4*sin(theta7) - C51*m5*sin(theta7) + C43*m4*cos(theta7)*cos(theta8) - L4*m5*cos(theta7)*cos(theta8) + C42*m4*cos(theta7)*sin(theta8) + C53*m5*cos(theta7)*cos(theta8)*cos(theta9) + C52*m5*cos(theta7)*cos(theta8)*sin(theta9) + C52*m5*cos(theta7)*cos(theta9)*sin(theta8) - C53*m5*cos(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R23*(C31*m3*cos(theta7) + C41*m4*cos(theta7) + C51*m5*cos(theta7) + C33*m3*sin(theta7) + C43*m4*cos(theta8)*sin(theta7) - L4*m5*cos(theta8)*sin(theta7) + C42*m4*sin(theta7)*sin(theta8) + C53*m5*cos(theta8)*cos(theta9)*sin(theta7) + C52*m5*cos(theta8)*sin(theta7)*sin(theta9) + C52*m5*cos(theta9)*sin(theta7)*sin(theta8) - C53*m5*sin(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R22*sin(theta6)*(C33*m3*cos(theta7) - C31*m3*sin(theta7) - C41*m4*sin(theta7) - C51*m5*sin(theta7) + C43*m4*cos(theta7)*cos(theta8) - L4*m5*cos(theta7)*cos(theta8) + C42*m4*cos(theta7)*sin(theta8) + C53*m5*cos(theta7)*cos(theta8)*cos(theta9) + C52*m5*cos(theta7)*cos(theta8)*sin(theta9) + C52*m5*cos(theta7)*cos(theta9)*sin(theta8) - C53*m5*cos(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob25 =
% (R23*cos(theta7)*(L4*m5*sin(theta8) + C52*m5*cos(theta8 + theta9) - C53*m5*sin(theta8 + theta9) + C42*m4*cos(theta8) - C43*m4*sin(theta8)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R22*(m4*(C43*cos(theta6)*cos(theta8) + C42*cos(theta6)*sin(theta8) + C42*cos(theta8)*sin(theta6)*sin(theta7) - C43*sin(theta6)*sin(theta7)*sin(theta8)) - m5*(L4*cos(theta6)*cos(theta8) - C53*cos(theta6)*cos(theta8)*cos(theta9) - C52*cos(theta6)*cos(theta8)*sin(theta9) - C52*cos(theta6)*cos(theta9)*sin(theta8) + C53*cos(theta6)*sin(theta8)*sin(theta9) - L4*sin(theta6)*sin(theta7)*sin(theta8) - C52*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) + C53*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9) + C53*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8) + C52*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R21*(m4*(C43*cos(theta8)*sin(theta6) + C42*sin(theta6)*sin(theta8) - C42*cos(theta6)*cos(theta8)*sin(theta7) + C43*cos(theta6)*sin(theta7)*sin(theta8)) + m5*(C53*cos(theta8)*cos(theta9)*sin(theta6) - L4*cos(theta8)*sin(theta6) + C52*cos(theta8)*sin(theta6)*sin(theta9) + C52*cos(theta9)*sin(theta6)*sin(theta8) - L4*cos(theta6)*sin(theta7)*sin(theta8) - C53*sin(theta6)*sin(theta8)*sin(theta9) - C52*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + C53*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + C53*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8) + C52*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob26 =
% -(m5*(C53*R23*sin(theta8 + theta9)*cos(theta7) - C52*R23*cos(theta8 + theta9)*cos(theta7) + C53*R22*cos(theta6)*cos(theta8)*cos(theta9) + C52*R22*cos(theta6)*cos(theta8)*sin(theta9) + C52*R22*cos(theta6)*cos(theta9)*sin(theta8) + C53*R21*cos(theta8)*cos(theta9)*sin(theta6) + C52*R21*cos(theta8)*sin(theta6)*sin(theta9) + C52*R21*cos(theta9)*sin(theta6)*sin(theta8) - C53*R22*cos(theta6)*sin(theta8)*sin(theta9) - C53*R21*sin(theta6)*sin(theta8)*sin(theta9) - C52*R21*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + C52*R22*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) + C53*R21*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + C53*R21*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8) + C52*R21*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9) - C53*R22*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9) - C53*R22*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8) - C52*R22*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob31 =
% (R31*cos(theta1)*(C33*m3*cos(theta2) - C31*m3*sin(theta2) - C41*m4*sin(theta2) - C51*m5*sin(theta2) + C43*m4*cos(theta2)*cos(theta3) - L4*m5*cos(theta2)*cos(theta3) + C42*m4*cos(theta2)*sin(theta3) + C53*m5*cos(theta2)*cos(theta3)*cos(theta4) + C52*m5*cos(theta2)*cos(theta3)*sin(theta4) + C52*m5*cos(theta2)*cos(theta4)*sin(theta3) - C53*m5*cos(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R33*(C31*m3*cos(theta2) + C41*m4*cos(theta2) + C51*m5*cos(theta2) + C33*m3*sin(theta2) + C43*m4*cos(theta3)*sin(theta2) - L4*m5*cos(theta3)*sin(theta2) + C42*m4*sin(theta2)*sin(theta3) + C53*m5*cos(theta3)*cos(theta4)*sin(theta2) + C52*m5*cos(theta3)*sin(theta2)*sin(theta4) + C52*m5*cos(theta4)*sin(theta2)*sin(theta3) - C53*m5*sin(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R32*sin(theta1)*(C33*m3*cos(theta2) - C31*m3*sin(theta2) - C41*m4*sin(theta2) - C51*m5*sin(theta2) + C43*m4*cos(theta2)*cos(theta3) - L4*m5*cos(theta2)*cos(theta3) + C42*m4*cos(theta2)*sin(theta3) + C53*m5*cos(theta2)*cos(theta3)*cos(theta4) + C52*m5*cos(theta2)*cos(theta3)*sin(theta4) + C52*m5*cos(theta2)*cos(theta4)*sin(theta3) - C53*m5*cos(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob32 =
% (R33*cos(theta2)*(L4*m5*sin(theta3) + C52*m5*cos(theta3 + theta4) - C53*m5*sin(theta3 + theta4) + C42*m4*cos(theta3) - C43*m4*sin(theta3)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R32*(m4*(C43*cos(theta1)*cos(theta3) + C42*cos(theta1)*sin(theta3) + C42*cos(theta3)*sin(theta1)*sin(theta2) - C43*sin(theta1)*sin(theta2)*sin(theta3)) - m5*(L4*cos(theta1)*cos(theta3) - C53*cos(theta1)*cos(theta3)*cos(theta4) - C52*cos(theta1)*cos(theta3)*sin(theta4) - C52*cos(theta1)*cos(theta4)*sin(theta3) + C53*cos(theta1)*sin(theta3)*sin(theta4) - L4*sin(theta1)*sin(theta2)*sin(theta3) - C52*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) + C53*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) + C53*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) + C52*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R31*(m4*(C43*cos(theta3)*sin(theta1) + C42*sin(theta1)*sin(theta3) - C42*cos(theta1)*cos(theta3)*sin(theta2) + C43*cos(theta1)*sin(theta2)*sin(theta3)) + m5*(C53*cos(theta3)*cos(theta4)*sin(theta1) - L4*cos(theta3)*sin(theta1) + C52*cos(theta3)*sin(theta1)*sin(theta4) + C52*cos(theta4)*sin(theta1)*sin(theta3) - L4*cos(theta1)*sin(theta2)*sin(theta3) - C53*sin(theta1)*sin(theta3)*sin(theta4) - C52*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C53*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C53*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) + C52*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob33 =
% -(m5*(C53*R33*sin(theta3 + theta4)*cos(theta2) - C52*R33*cos(theta3 + theta4)*cos(theta2) + C53*R32*cos(theta1)*cos(theta3)*cos(theta4) + C52*R32*cos(theta1)*cos(theta3)*sin(theta4) + C52*R32*cos(theta1)*cos(theta4)*sin(theta3) + C53*R31*cos(theta3)*cos(theta4)*sin(theta1) + C52*R31*cos(theta3)*sin(theta1)*sin(theta4) + C52*R31*cos(theta4)*sin(theta1)*sin(theta3) - C53*R32*cos(theta1)*sin(theta3)*sin(theta4) - C53*R31*sin(theta1)*sin(theta3)*sin(theta4) - C52*R31*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2) + C52*R32*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2) + C53*R31*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + C53*R31*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) + C52*R31*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4) - C53*R32*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - C53*R32*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) - C52*R32*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob34 =
% (R31*cos(theta6)*(C33*m3*cos(theta7) - C31*m3*sin(theta7) - C41*m4*sin(theta7) - C51*m5*sin(theta7) + C43*m4*cos(theta7)*cos(theta8) - L4*m5*cos(theta7)*cos(theta8) + C42*m4*cos(theta7)*sin(theta8) + C53*m5*cos(theta7)*cos(theta8)*cos(theta9) + C52*m5*cos(theta7)*cos(theta8)*sin(theta9) + C52*m5*cos(theta7)*cos(theta9)*sin(theta8) - C53*m5*cos(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R33*(C31*m3*cos(theta7) + C41*m4*cos(theta7) + C51*m5*cos(theta7) + C33*m3*sin(theta7) + C43*m4*cos(theta8)*sin(theta7) - L4*m5*cos(theta8)*sin(theta7) + C42*m4*sin(theta7)*sin(theta8) + C53*m5*cos(theta8)*cos(theta9)*sin(theta7) + C52*m5*cos(theta8)*sin(theta7)*sin(theta9) + C52*m5*cos(theta9)*sin(theta7)*sin(theta8) - C53*m5*sin(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R32*sin(theta6)*(C33*m3*cos(theta7) - C31*m3*sin(theta7) - C41*m4*sin(theta7) - C51*m5*sin(theta7) + C43*m4*cos(theta7)*cos(theta8) - L4*m5*cos(theta7)*cos(theta8) + C42*m4*cos(theta7)*sin(theta8) + C53*m5*cos(theta7)*cos(theta8)*cos(theta9) + C52*m5*cos(theta7)*cos(theta8)*sin(theta9) + C52*m5*cos(theta7)*cos(theta9)*sin(theta8) - C53*m5*cos(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob35 =
% (R33*cos(theta7)*(L4*m5*sin(theta8) + C52*m5*cos(theta8 + theta9) - C53*m5*sin(theta8 + theta9) + C42*m4*cos(theta8) - C43*m4*sin(theta8)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R32*(m4*(C43*cos(theta6)*cos(theta8) + C42*cos(theta6)*sin(theta8) + C42*cos(theta8)*sin(theta6)*sin(theta7) - C43*sin(theta6)*sin(theta7)*sin(theta8)) - m5*(L4*cos(theta6)*cos(theta8) - C53*cos(theta6)*cos(theta8)*cos(theta9) - C52*cos(theta6)*cos(theta8)*sin(theta9) - C52*cos(theta6)*cos(theta9)*sin(theta8) + C53*cos(theta6)*sin(theta8)*sin(theta9) - L4*sin(theta6)*sin(theta7)*sin(theta8) - C52*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) + C53*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9) + C53*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8) + C52*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5) - (R31*(m4*(C43*cos(theta8)*sin(theta6) + C42*sin(theta6)*sin(theta8) - C42*cos(theta6)*cos(theta8)*sin(theta7) + C43*cos(theta6)*sin(theta7)*sin(theta8)) + m5*(C53*cos(theta8)*cos(theta9)*sin(theta6) - L4*cos(theta8)*sin(theta6) + C52*cos(theta8)*sin(theta6)*sin(theta9) + C52*cos(theta9)*sin(theta6)*sin(theta8) - L4*cos(theta6)*sin(theta7)*sin(theta8) - C53*sin(theta6)*sin(theta8)*sin(theta9) - C52*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + C53*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + C53*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8) + C52*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9))))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 
% jacob36 =
% -(m5*(C53*R33*sin(theta8 + theta9)*cos(theta7) - C52*R33*cos(theta8 + theta9)*cos(theta7) + C53*R32*cos(theta6)*cos(theta8)*cos(theta9) + C52*R32*cos(theta6)*cos(theta8)*sin(theta9) + C52*R32*cos(theta6)*cos(theta9)*sin(theta8) + C53*R31*cos(theta8)*cos(theta9)*sin(theta6) + C52*R31*cos(theta8)*sin(theta6)*sin(theta9) + C52*R31*cos(theta9)*sin(theta6)*sin(theta8) - C53*R32*cos(theta6)*sin(theta8)*sin(theta9) - C53*R31*sin(theta6)*sin(theta8)*sin(theta9) - C52*R31*cos(theta6)*cos(theta8)*cos(theta9)*sin(theta7) + C52*R32*cos(theta8)*cos(theta9)*sin(theta6)*sin(theta7) + C53*R31*cos(theta6)*cos(theta8)*sin(theta7)*sin(theta9) + C53*R31*cos(theta6)*cos(theta9)*sin(theta7)*sin(theta8) + C52*R31*cos(theta6)*sin(theta7)*sin(theta8)*sin(theta9) - C53*R32*cos(theta8)*sin(theta6)*sin(theta7)*sin(theta9) - C53*R32*cos(theta9)*sin(theta6)*sin(theta7)*sin(theta8) - C52*R32*sin(theta6)*sin(theta7)*sin(theta8)*sin(theta9)))/(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5)
% 