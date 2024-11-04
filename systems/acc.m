%% CRUISE CONTROL: CONSTANT DISTANCE
% Parameters
m_c = 1650; % Mass of the follower car [kg]
T_s = 0.2; % Sampling time [s]
delay_steps = 3;
u_min = -2000; % Minimum control input [N]
u_max = 2000; % Maximum control input [N]
N_p = 15; % Prediction horizon

% State-space matrices
A = [0 1;
    0 0];
B = [0; 1/m_c];
C = [1 0];
D = 0;

sys_cont = ss(A, B, C, D);

% Discretize the system
sys_disc = c2d(sys_cont, T_s);
[Ad, Bd, Cd, Dd] = ssdata(sys_disc);

% Create a state-space model with delay
plant = delayss(Ad, Bd, Cd, Dd, T_s, delay_steps);

% Initialize the MPC controller
mpc_controller = mpc(plant, T_s, N_p, 2);
mpc_controller.MV.Delay = delay_steps;

% Set constraints on the manipulated variable
mpc_controller.MV.Min = u_min;
mpc_controller.MV.max = u_max;

% MPC tuning parameters
mpc_controller.Weights.ManipulatedVariablesRate = 0.01; % Penalize control changes
mpc_controller.Weights.OutputVariables = 1; % Penalize distance variation
mpc_controller.Weights.ManipulatedVariables = 0; % No penalty on input value

% Initial conditions and target setup
initial_distance = 1;
initial_relative_velocity = 0;
x = [initial_distance; initial_rel_velocity]; % Initial state
target_distance = 5;
x_history = zeros(2, N_p);
u_history = zeros(1, N_p);

% Initialize the MPC state
x_mpc = mpcstate(mpc_controller);

% Simulation loop
for k=1:N_p
    r = target_distance; % Reference signal
    u = mpcmove(mpc_controller, x_mpc, x, r); % Compute optimal control action
    x = Ad*x + Bd*u; % Apply optimal control action
    x_history(:, k) = x; % Log states
    u_history(k) = u; % Log control input
end

% Plot the results
time = (0:N_p) * T_s;

figure; hold on;
subplot(3, 1, 1);
plot(time, x_history(1,:), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Distance Variation (m)');
title('Distance Evolution between Follower & Leader Cars');

subplot(3, 1, 2); hold on;
plot(time, x_history(2,:), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Relative Velocity (m/s)');
title('Relative Velocity between Follower & Leader Cars');

subplot(3, 1, 3); hold on;
plot(time, u_history, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (N)');
title('Control Input (Engine Force)')
