%% Step 0: Setup
toggle = struct( ...
                'debug', true, ...
                'save', true, ...
                'discretize', false ...
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
                    'target_penalty', true, ...
                    'verbose', false ...
                   );

%% Step 1: Define and import parameters
sys = ddsf_systems("cruise_control", toggle.discretize); 

dims = sys.dims;
T_sim = 25;
datagen_mode = 'scaled_gaussian';
lookup = struct( ...
                'sys', sys, ...
                'sys_params', sys.params, ...
                'opt_params', opt_params, ...
                'config', sys.config, ...
                'dims', dims, ...
                'IO_params', IO_params, ...
                'T_sim', T_sim, ...
                'datagen_mode', datagen_mode ...
                );
% DEBUG STATEMENTS
%lookup.sys.S_f.u_eq = 0;
%lookup.sys.S_f.y_eq = lookup.sys.params.target;


%% Step 2: Generate data & Hankel matrices
[u_d, y_d, x_d, ~, ~] = ddsfGenerateData(lookup); 

[H_u, H_y] = ddsf_hankel(u_d, y_d, sys);
 
lookup.H = [H_u; H_y];
lookup.H_u = H_u; lookup.H_y = H_y;
lookup.dims.hankel_cols = size(H_u, 2);

% START DEBUG STATEMENTS
% lookup.config.R = 150;
%lookup.config.T = 100;
epsilon = 0.01;
lookup.H_u = H_u + epsilon * eye(size(H_u)); 
lookup.H_y = H_y + epsilon * eye(size(H_y));
lookup.H = [H_u; H_y];
% END DEBUG STATEMENTS

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

    u_l = learning_policy(lookup);
    %fprintf("Submitting u_l with value: "); disp(u_l);

    [u_opt, y_opt] = ddsf_opt(lookup, u_l, traj_ini);
    %fprintf("Received optimal values u_opt = %d, y_opt = %d\n", value(u_opt), value(y_opt));

    u_next = u_opt(:, 1);
    y_next = y_opt(:, 1);
    %fprintf("Storing first optimal values u_opt[1] = %d, y_opt[1] = %d\n", value(u_opt(:, 1)), value(y_opt(:, 1)));
    
    logs.u_d(:, t) = u_l(:, 1);
    logs.u(:, t) = u_next;
    logs.y(:, t) = y_next;

    % Add recursively feasible points to the safe terminal set
    lookup.sys.S_f.u_eq = [lookup.sys.S_f.u_eq, u_next];
    lookup.sys.S_f.y_eq = [lookup.sys.S_f.y_eq, y_next];
end

%% Plot the results
time = 0:(T_sim + lookup.config.T_ini);
ddsf_plot(time, logs, lookup)

% Should this use the same policy as data generation?
function u_l = learning_policy(lookup)
    sys = lookup.sys;
    m = sys.dims.m;
    L = lookup.config.N_p + 2 * lookup.config.T_ini;
    mode = lookup.datagen_mode;

    lb = sys.constraints.U(:, 1);
    lb(lb == -inf) = 1;
    ub = sys.constraints.U(:, 2);
    ub(ub == inf) = 1;
    %fprintf("1. <learning_policy> Lower and Upper bounds: [%d, %d]\n", lb, ub);

    switch mode
        case 'scaled_gaussian'
            scale = 1.25;
            ub = (ub > 0) .* (scale .* ub) + (ub < 0) .* ((scale^(-1)) .* ub) + (ub == 0) .* (10^scale);
            lb = (lb < 0) .* (scale .* lb) + (lb > 0) .* ((scale^(-1)) .* lb) + (lb == 0) .* (- 10^scale);
            ub = (ub > 0) .* (scale .* ub) + (ub < 0) .* ((scale^(-1)) .* ub);
            lb = (lb < 0) .* (scale .* lb) + (lb > 0) .* ((scale^(-1)) .* lb);
            u_l = lb + (ub - lb) .* rand(m, L);
            % DEBUG STATEMENT
            % u_l = ones(m, L);
            %fprintf("2. <learning_policy> Relaxed lower and Upper bounds: [%d, %d]\n", lb, ub);
        case 'rbs'
            u_l = idinput([m, L], 'rbs', [0, 1], [-1,1]).'; 
    end
end

%   Can later be changed to:
%       u_l = learning_policy(y, y_d)
%
%   INPUTS:
%       y   - Current system output (px1)
%       y_d - Desired system output
