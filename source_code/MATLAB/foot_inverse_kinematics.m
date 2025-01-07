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
for i = 1:10
    S{i} = [0 -W{i}(3) W{i}(2) V{i}(1);
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
T_left = simplify(expm(S{1}*theta1)*expm(S{2}*theta2)*expm(S{3}*theta3)*expm(S{4}*theta4)*expm(S{5}*theta5)*M_left);
T_right = simplify(expm(S{6}*theta6)*expm(S{7}*theta7)*expm(S{8}*theta8)*expm(S{9}*theta9)*expm(S{10}*theta10)*M_right);

% value substitution
val_theta1 = 0;
val_theta2 = 0.0001;
val_theta3 = 0.0001;
val_theta4 = 0.0001;
val_theta5 = 0;
val_theta6 = 0;
val_theta7 = pi/2;
val_theta8 = 0;
val_theta9 = 0;
val_theta10 = 0;
val_L1 = 0.135;
val_L2 = 0.12;
val_L3 = 0.09;
val_L4 = 0.18;
val_L5 = 0.18;
val_L6 = 0.075;
sub_vals = [val_theta1 val_theta5 val_theta6 val_theta7 val_theta8 val_theta9 val_L1 val_L2 val_L3 val_L4 val_L5 val_L6];
sub_syms = [theta1 theta5 theta6 theta7 theta8 theta9 L1 L2 L3 L4 L5 L6];

% coordinate vector
x = T_left(1,4);
y = T_left(2,4);
z = T_left(3,4);
sym_X_act = [x;y;z];
X_ref = [-0.135;0.01;-0.55];

% Newton Raphson Method
sym_theta = [theta2 theta3 theta4];
val_theta = [val_theta2 val_theta3 val_theta4];

norm_e = 1;
epsilon = 0.005;
iteration = 0;
jacob = simplify(jacobian(sym_X_act,sym_theta))
while norm_e > epsilon && iteration < 100
    val_X_act = vpa(subs(sym_X_act,[sym_theta sub_syms],[val_theta sub_vals]),8);
    error_X = vpa(X_ref - val_X_act,8);
    norm_e = norm(error_X);
    val_jacob = vpa(subs(jacob,[sym_theta sub_syms],[val_theta sub_vals]),8);
    inv_jacob = inv(val_jacob);
    delta_theta = val_jacob\error_X;
    val_theta = val_theta + delta_theta';
    val_theta = mod(eval(val_theta) + pi, 2*pi) - pi;
    iteration = iteration + 1;
end
val_theta
% leg Jacobian matrix for Orin
% jacob(1,1) = -cos(theta1)*cos(theta2)*(L5*cos(theta3 + theta4) + L4*cos(theta3) + L6*cos(theta3 + theta4 + theta5))
% jacob(1,2) = L4*cos(theta3)*sin(theta1) + L5*cos(theta3)*cos(theta4)*sin(theta1) + L4*cos(theta1)*sin(theta2)*sin(theta3) - L5*sin(theta1)*sin(theta3)*sin(theta4) + L6*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1) + L5*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + L5*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) - L6*cos(theta3)*sin(theta1)*sin(theta4)*sin(theta5) - L6*cos(theta4)*sin(theta1)*sin(theta3)*sin(theta5) - L6*cos(theta5)*sin(theta1)*sin(theta3)*sin(theta4) + L6*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5) + L6*cos(theta1)*cos(theta3)*cos(theta5)*sin(theta2)*sin(theta4) + L6*cos(theta1)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta3) - L6*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5)
% jacob(1,3) = L5*cos(theta3)*cos(theta4)*sin(theta1) - L5*sin(theta1)*sin(theta3)*sin(theta4) + L6*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta1) + L5*cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4) + L5*cos(theta1)*cos(theta4)*sin(theta2)*sin(theta3) - L6*cos(theta3)*sin(theta1)*sin(theta4)*sin(theta5) - L6*cos(theta4)*sin(theta1)*sin(theta3)*sin(theta5) - L6*cos(theta5)*sin(theta1)*sin(theta3)*sin(theta4) + L6*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5) + L6*cos(theta1)*cos(theta3)*cos(theta5)*sin(theta2)*sin(theta4) + L6*cos(theta1)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta3) - L6*cos(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5)
% jacob(2,1) = cos(theta2)*sin(theta1)*(L5*cos(theta3 + theta4) + L4*cos(theta3) + L6*cos(theta3 + theta4 + theta5))
% jacob(2,2) = L4*cos(theta1)*cos(theta3) + L5*cos(theta1)*cos(theta3)*cos(theta4) - L5*cos(theta1)*sin(theta3)*sin(theta4) - L4*sin(theta1)*sin(theta2)*sin(theta3) + L6*cos(theta1)*cos(theta3)*cos(theta4)*cos(theta5) - L6*cos(theta1)*cos(theta3)*sin(theta4)*sin(theta5) - L6*cos(theta1)*cos(theta4)*sin(theta3)*sin(theta5) - L6*cos(theta1)*cos(theta5)*sin(theta3)*sin(theta4) - L5*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - L5*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) - L6*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta5) - L6*cos(theta3)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta4) - L6*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3) + L6*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5)
% jacob(2,3) = L5*cos(theta1)*cos(theta3)*cos(theta4) - L5*cos(theta1)*sin(theta3)*sin(theta4) + L6*cos(theta1)*cos(theta3)*cos(theta4)*cos(theta5) - L6*cos(theta1)*cos(theta3)*sin(theta4)*sin(theta5) - L6*cos(theta1)*cos(theta4)*sin(theta3)*sin(theta5) - L6*cos(theta1)*cos(theta5)*sin(theta3)*sin(theta4) - L5*cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4) - L5*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta3) - L6*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)*sin(theta5) - L6*cos(theta3)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta4) - L6*cos(theta4)*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3) + L6*sin(theta1)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5)
% jacob(3,1) = sin(theta2)*(L5*cos(theta3 + theta4) + L4*cos(theta3) + L6*cos(theta3 + theta4 + theta5))
% jacob(3,2) = cos(theta2)*(L5*sin(theta3 + theta4) + L4*sin(theta3) + L6*sin(theta3 + theta4 + theta5))
% jacob(3,3) = (L5*sin(theta3 - theta2 + theta4))/2 + (L6*sin(theta2 + theta3 + theta4 + theta5))/2 + (L6*sin(theta3 - theta2 + theta4 + theta5))/2 + (L5*sin(theta2 + theta3 + theta4))/2

% The Jacobian of two legs are identical!

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
    T(:,:,i) = [cos(dhparams (i,1)) -sin(dhparams(i,1)) 0 dhparams(i,3);
        sin(dhparams(i,1))*cos(dhparams(i,4)) cos(dhparams(i,1))*cos(dhparams(i,4)) -sin(dhparams(i,4)) -dhparams(i,2)*sin(dhparams(i,4));
        sin(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,1))*sin(dhparams(i,4)) cos(dhparams(i,4)) dhparams(i,2)*cos(dhparams(i,4));
        0 0 0 1];
    T_05 = T_05*T(:,:,i);
end

T_05 = simplify(T_05);
x = T_05(1,4);
y = T_05(2,4);
z = T_05(3,4);

sym_X_act = [x;y;z];
sym_theta = [theta2 theta3 theta4];
sym_sub = [a1 a2 a3 a4 a5 alpha2 alpha3];
val_sub = [-0.135 -0.095 -0.09 -0.18 -0.38 -pi/2 -pi/2];

X_ref = [-0.23;0.001;0.60];
val_theta = [pi/2;0.001;0.001];
epsilon = 1;
error_X = 1;
iteration = 0;
while norm(error_X) > 0.002 && iteration < 100
    val_X_act = vpa(subs(sym_X_act,[sym_theta sym_sub theta1],[val_theta' val_sub 0]),2);
    jacob = vpa(subs(jacobian(sym_X_act,sym_theta),[sym_theta sym_sub theta1],[val_theta' val_sub 0]),2);
    inv_jacob = inv(jacob);
    error_X = vpa(X_ref - val_X_act,2);
    delta_theta = inv_jacob*error_X;
    norm(error_X)
    val_theta = val_theta + delta_theta;
    epsilon = abs(norm(eval(delta_theta)))/abs(norm(eval(val_theta)));
    iteration = iteration + 1;
end


val_theta = mod(eval(val_theta) + pi, 2*pi) - pi
val_X_act
norm(error_X);
epsilon