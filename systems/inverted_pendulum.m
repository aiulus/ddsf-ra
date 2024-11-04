% Parameters
M = 0.5; % mass of the cart [kg]
m = 0.2; % mass of the pendulum [kg]
l = 0.3; % Length of the pendulum [m]
g = 9.81; % Acceleration due to gravity [m/s^2]
dt = 0.01; % time step for discretization

% Continuous-time State-Space Matrices
A = [0 1 0 0;
    0 0 (-m*g/M) 0;
    0 0 0 1;
    0 0 (M+m)*g/(M*l) 0];
B = [0;
    1/M;
    0;
    -1/(M*l)];
C = [1 0 0 0;
    0 0 1 0];
D = [0; 0];

% Discretize the system
sys = ss(A, B, C, D);
sysd = c2d(sys, dt);

