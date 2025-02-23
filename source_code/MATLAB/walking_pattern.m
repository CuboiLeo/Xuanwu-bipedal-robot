clear; clc; close all;

sx = [0 0.3 0.3 0.3 0.3 0.2 0.4 0];      % [m] nominal step offsets in x
sy = [0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2];  % [m] nominal step offsets in y

zc = 0.4;      % [m] CoM height
g  = 9.81;     % [m/s^2]
Tc = sqrt(zc/g);

Tsup = 0.8;    % [s] support period
C = cosh(Tsup/Tc);
S = sinh(Tsup/Tc);

% Cost function weights in Eq. (4.59)
a = 10;
b = 1;
D = a*(C-1)^2 + b*(S/Tc)^2;

N = length(sx)-1;  % number of steps

% Initial CoM condition
x_local = 0;   
y_local = 0;   
xdot_local = 0; 
ydot_local = 0;

% Global foot positions for plotting
footX = zeros(1, N+1);
footY = zeros(1, N+1);
footX(1) = 0;    % first foot at (0,0) globally
footY(1) = 0;

% Global CoM positions for plotting
CoMX = zeros(1, N+1);
CoMY = zeros(1, N+1);
CoMX(1) = x_local;
CoMY(1) = y_local;

% Nominal position
px = 0;
py = 0;

% Modified position
px_star = 0;
py_star = 0;

for n = 1:N
    x_end    =  C*x_local + Tc*S*xdot_local;
    xdot_end = (S/Tc)*x_local + C*xdot_local;
    y_end    =  C*y_local + Tc*S*ydot_local;
    ydot_end = (S/Tc)*y_local + C*ydot_local;
    
    x_local    = x_end;
    xdot_local = xdot_end;
    y_local    = y_end;
    ydot_local = ydot_end;

    px = sx(n);
    py = - sy(n)*(-1)^n;

    x_bar = sx(n+1)/2;
    y_bar = sy(n+1)/2*(-1)^n;
    vx_bar = (C+1)/(Tc*S)*x_bar;
    vy_bar = (C-1)/(Tc*S)*y_bar;

    xd = px + x_bar;
    xd_dot = vx_bar;
    yd = py + y_bar;
    yd_dot = vy_bar;

    px_star = - a*(C - 1)/D * ( xd - C*x_local - Tc*S*xdot_local ) ...
              - b*(S/Tc)/D * ( xd_dot - (S/Tc)*x_local - C*xdot_local )
          
    py_star = - a*(C - 1)/D * ( yd - C*y_local - Tc*S*ydot_local ) ...
              - b*(S/Tc)/D * ( yd_dot - (S/Tc)*y_local - C*ydot_local )
    
    x_local = x_local - px_star;
    y_local = y_local - py_star;

    footX(n+1) = footX(n) + px_star;
    footY(n+1) = footY(n) + py_star;
    CoMX(n+1) = footX(n+1) + x_local;
    CoMY(n+1) = footY(n+1) + y_local;
end

%% 4) Plotting results
figure('Color','white'); hold on; grid on;
plot(footX, footY, 'rs--','LineWidth',2,'MarkerFaceColor','r',...
    'DisplayName','Foot Placements');
plot(CoMX, CoMY, 'bo-','LineWidth',1.5,...
    'DisplayName','CoM (end of each step)');
legend('Location','best');
axis equal;
title('Kajita 3D-LIPM Walking (Local Computation + Global Plot)');
xlabel('X [m]'); ylabel('Y [m]');
