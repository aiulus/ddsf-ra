%% Step 0: Setup
toggle = struct( ...
                'debug', true, ...
                'save', true, ...
                'discretize', 'true' ...
                );

IO_params = struct( ...
    'debug_toggle', toggle.debug, ...
    'save_to_file', toggle.save, ...
    'log_interval', 1 ... % Log every n-the step of the simulation
    );

opt_params = struct( ...
                    'regularize', true, ...
                    'constr_type', 'f', ...
                    'solver_type', 'o', ...
                    'verbose', false ...
                   );

%% Step 1: Define and import parameters
sys = ddsf_systems("cruise_control", toggle.discretize); 

dims = sys.dims;
T_sim = 25;
lookup = struct( ...
                'sys', sys, ...
                'sys_params', sys.params, ...
                'config', sys.config, ...
                'dims', dims, ...
                'IO_params', IO_params, ...
                'T_sim', T_sim ...
                );

% DEBUG STATEMENTS
lookup.config.R = 1;
lookup.config.T = 100;

%% Step 2: Generate data & Hankel matrices
[u_d, y_d, x_d, ~, ~] = ddsfGenerateData(lookup); 

[H_u, H_y] = ddsf_hankel(u_d, y_d, sys);
 
lookup.H = [H_u; H_y];
lookup.H_u = H_u; lookup.H_y = H_y;
lookup.dims.hankel_cols = size(H_u, 2);

%% Initialize objects to log simulation history
logs = struct( ...
        'u_d', zeros(dims.m, T_sim + lookup.config.T_ini), ...
        'u', [u_d(:, end - lookup.config.T_ini + 1: end).'; ...
        zeros(dims.m, T_sim).'].', ... 
        'y', [y_d(:, end - lookup.config.T_ini + 1: end).'; ...
        zeros(dims.p, T_sim).'].', ... 
        'x', [x_d(:, end - lookup.config.T_ini + 1: end).'; ...
        zeros(dims.n, T_sim).'].' ... % TODO: tracking x not necessary
    );

%% Step 3: Receding Horizon Loop
for t=(lookup.config.T_ini+1):(lookup.config.T_ini + 1 + T_sim)
    fprintf("----------------- DEBUG Information -----------------\n");
    fprintf("CURRENT SIMULATION STEP: t = %d\n", t - lookup.config.T_ini);

    u_ini = logs.u(:, (t - lookup.config.T_ini):(t-1));
    y_ini = logs.y(:, (t -lookup.config.T_ini):(t-1));
    traj_ini = [u_ini; y_ini];
    u_l = learning_policy();

    [u_opt, y_opt] = ddsf_opt(lookup, u_l, traj_ini, opt_params);
    u_next = u_opt(:, 1);
    y_next = y_opt(:, 1);
    
    logs.u_d(:, t) = u_l;
    logs.u(:, t) = u_next;
    logs.y(:, t) = y_next;
end

%% Plot the results
time = 0:(T_sim + 1);
ddsf_plot(time, logs, sys)

function u_l = learning_policy()
    %   LATER change to:
    %       u_l = learning_policy(y, y_d)
    %
    %   INPUTS:
    %       y   - Current system output (px1)
    %       y_d - Desired system output

    %u_l = randi([-1, 1], 1, 1);
    prbs = idinput(1, 'prbs', [], [-1, 1]);

    lb = - 1.5;
    ub = 1.5;
    random_magnitude = lb + (ub - lb) * rand;
    
    % Returns a pseudo-random binary signal scaled by a random magnitude
    u_l = random_magnitude * prbs * 5000;
end

