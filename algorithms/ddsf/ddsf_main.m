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

%% Step 1: Data collection
[u_d, y_d, ~, ~] = generate_data(sys, T); % Simulate the system

%% Step 2: Generate the Hankel matrices
[Up, Yp, Uf, Yf] = deepc_hankel(u_d, y_d, sys);