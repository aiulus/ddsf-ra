%% Data-Driven Safety Filter 
debug_toggle = true; % Toggle debug mode
save_to_file = true; % Set to true if log file desired
log_interval = 1; % Log every <log_interval> iterations

%% Step 0: Define global parameters, fetch system
sys = linear_system("cruise_control"); % Fetch system
config = sys.ddsf_config;

T = config.T; % # Data points
T_ini = config.T_ini; % Length of initial trajectory
N_p = config.N_p; % Prediction horizon
s = config.s; % Sliding length
Q = config.Q * eye(sys.params.p); % Output cost matrix
R = config.R * eye(sys.params.m); % Control cost matrix

m = sys.params.m;  % Input dimension
p = sys.params.p; % Output dimension
n = sys.params.n; % Dim. of the minimal state-space representation

%% Step 1: Offline setup
% Sample data from the system
[u_d, y_d, ~, ~] = generate_data(sys, T); 

% Compute the initial trajectory
t = 0; % Time step

% Generate the Hankel matrices
[H_u, H_y] = deepc_hankel(u_d, y_d, sys);
max_iter = 50;
l_process_done = false; % TODO: This is a temporary placeholder representing
                        %       whether the l. process has converged

%% Step 2: Online Loop
while true 
    if t > max_iter || l_process_done
        break;
    end
    traj_ini = update_initial_trajectory(u_d, y_d, t);
    [alpha, u_opt, y_opt] = ddsf_opt(u_l, H_u, H_y, traj_ini, sys);
    %% TODO: Apply u_opt to the system
    %% Feed both y_opt to the l. algorithm
end

function traj_ini = update_initial_trajectory(u_d, y_d, t)
    u_ini = u_d(:, (1 + t):(T_ini + t));
    y_ini = y_d(:, (1 + t):(T_ini + t));
    traj_ini = [u_ini; y_ini];
end