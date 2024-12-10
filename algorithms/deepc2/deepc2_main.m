%% Define the system
sys = deepc2_systems("example0");
verbose = false; 
rng(0, 'twister'); % Set seed and generator

%% Define simulation parameters
run_config = struct( ...
    'T_ini', 5, ... % Initial trajectory length
    'T_f', 4, ... % Prediction horizon
    'T_sim', 10 ... % Number of data points
    );

dims = struct( ...
    'n', size(sys.A, 1), ... % System state dim.
    'm', size(sys.B, 2), ... % Input dim.
    'p', size(sys.C, 1), ... % Output shape
    'q', size(sys.B, 2) + size(sys.C, 1) ... % #I/O variables
    );

%% Initialize DeePC parameters
run_config.L = run_config.T_ini + run_config.T_f;
% TODO: Check / Find out why there is a hard-coding
run_config.T = (dims.m * dims.n + dims.m+1)*(run_config.L + dims.n) + 30;

deepc_params = struct( ...
    'Q', eye(dims.p), ... % Output cost matrix
    'R', eye(dims.m), ... % Control cost matrix
    'lambda_g', 0 ...
    );

%% Initialize containers for logging
u_sim = zeros(dims.m, run_config.T_sim);
y_sim = zeros(dims.p, run_config.T_sim);

u_data = zeros(dims.m, run_config.T);
y_data = zeros(dims.p, run_config.T);
x_data = zeros(dims.n, run_config.T + 1);
x0 = rand(dims.n, 1); % Random initial state
x_data(:, 1) = x0;
x = x0;

%% Parameters for Hankel matrix construction
hankel_params = struct( ...
    'u_data', u_data, ...
    'x_data', x_data, ...
    'y_data', y_data, ...
    'T_ini', run_config.T_ini, ...
    'T_f', run_config.T_f, ...
    'n', dims.n ...
    );

%% Generate data
for t = 1:run_config.T
    u = rand(dims.m, 1);
    y = sys.C* x + sys.D * u;
    x = sys.A * x + sys.B * u;
    u_data(:, t) = u;
    y_data(:, t) = y;
    x_data(:, t + 1) = x;
end

data = struct( ...
     'u_data', u_data, ...
     'y_data', y_data, ...
     'x0', x0, ...
     'x_data', x_data);

%% Data structure that holds all relevant parameters
lookup = struct( ...
    'dims', dims, ...
    'hankel', hankel_params, ...
    'config', run_config, ...
    'deepc', deepc_params, ...
    'sys', sys, ...
    'data', data ...
    );

H = deepc2_hankel(lookup);

%% Initialize simulation results
ypre1 = zeros(dims.p, run_config.T_sim);
ypre2 = zeros(dims.p, run_config.T_sim);
ypre3 = zeros(dims.p, run_config.T_sim);
ypre4 = zeros(dims.p, run_config.T_sim);

% Extract the last column of xData and reshape it into a column vector
x = reshape(x_data(:, end), [], 1);

% Reshape the last Tini columns of PData, uData, and yData into column vectors
u_ini = reshape(u_data(:, end - run_config.T_ini + 1:end), [], 1);
y_ini = reshape(y_data(:, end - run_config.T_ini + 1:end), [], 1);

%% Receding Horizon Loop
for t=1:run_config.T_sim
    % Solve the quadratic optimization problem
    [u, y_p] =  deepc2_get_input(lookup, H, u_ini, y_ini);
    
    % Apply system dynamics
    y = sys.C * x + sys.D * u;
    x = sys.A * x + sys.B * u;
    
    % Log the resulting trajectory
    u_sim(:, t) = u;
    y_sim(:, t) = y;

    ypre1(:, t) = y_p(1:dims.p);
    ypre2(:, t) = y_p(dims.p + 1: 2 * dims.p);
    ypre3(:, t) = y_p(2 * dims.p + 1: 3 * dims.p);
    ypre4(:, t) = y_p(3 * dims.p + 1: end);
    
    % Update the initial trajectory
    u_ini = [u_ini(dims.m + 1:end, :); u];
    y_ini = [y_ini(dims.p + 1:end, :); y];

            fprintf("------------------- " + ...
            "Simulation step %d -------------------\n", t);

    if verbose
        fprintf("Optimal input: "); disp(u);
        fprintf("Corresp. output: "); disp(y);
    end
end

%% Plot the results
Tsim = run_config.T_sim; 
time = 0:Tsim-1;
deepc2_plot(time, y_sim, u_sim, ypre1, ypre2, ypre3, ypre4)

%% Save the results
% deepc2_save(time, u_sim, y_sim)

function [u_opt, y_opt] = deepc2_get_input(lookup, H, u_ini, y_ini)
    m = lookup.dims.m;
    [u, y] = deepc2Opt(lookup, H, u_ini, y_ini);
    u_opt = u(1:m);
    y_opt = y.';
end

function deepc2_save(time, u_sim, y_sim)
    u_csv = [time(:), u_sim(1, :).'];
    y_csv = [time(:), y_sim(1, :).'];
    writematrix(u_csv, 'u.csv');
    writematrix(y_csv, 'y.csv');
end






