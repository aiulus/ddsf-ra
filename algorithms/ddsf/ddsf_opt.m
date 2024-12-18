function [u_opt, y_opt] = ddsf_opt(lookup, u_l, traj_ini, opt_params)
    %% Extract parameters
    % Lengths and dimensions
    T_ini = lookup.config.T_ini;
    N_p = lookup.config.N_p;
    L = N_p + 2 * T_ini;
    m = lookup.dims.m;
    p = lookup.dims.p;
    num_cols = lookup.dims.hankel_cols;
    
    % Matrices
    R = lookup.config.R;
    H = lookup.H;
    
    % Constraints
    U = lookup.sys.constraints.U;
    Y = lookup.sys.constraints.Y;
    u_min = U(:, 1);
    u_max = U(:, 2);
    y_min = Y(:, 1);
    y_max = Y(:, 2);

    % Initial trajectory and equilibrium states
    u_ini = traj_ini(1:m, :);
    y_ini = traj_ini(m+1:end, :);
    u_eq = lookup.sys.S_f.u_eq;
    y_eq = lookup.sys.S_f.y_eq;

    if opt_params.regularize
        H = ddsf_regularize_hankel(lookup.H_u, lookup.H_y);
        num_cols = size(H, 2);
    end

    %% Define symbolic variables
    alpha = sdpvar(num_cols, 1);
    control_u = sdpvar(m, N_p + 2 * T_ini);
    control_y = sdpvar(p, N_p + 2 * T_ini);
    % traj_p = [control_u; control_y];
    
    %% Populate the initial and terminal parts of the trajectory
    % Replaces the encodings in constraints
    control_u(:, 1:T_ini) = u_ini;
    control_y(:, 1:T_ini) = y_ini;
    control_u(:, T_ini + N_p + 1 : end) = repmat(u_eq, 1, T_ini);
    control_y(:, T_ini + N_p + 1 : end) = repmat(y_eq, 1, T_ini);

    %% Flatten the variables 
    u_bar = reshape(control_u.', [], 1);
    y_bar = reshape(control_y.', [], 1);
    traj_p_bar = [u_bar; y_bar];


    %% Define the objective function and the constraints
    delta_u = control_u(:, 1) - u_l;
    objective = delta_u.' * R * delta_u;

    constraints = [traj_p_bar == H * alpha]; 
    
    switch opt_params.constr_type
        case 's' % Just enforce system behavior
            % No additional constraints needed
        case 'u' % Just encode input constraints
            constraints = [constraints, ...
                           control_u >= repmat(u_min, 1, N_p + 2 * T_ini), ...
                           control_u <= repmat(u_max, 1, N_p + 2 * T_ini)];
        case 'y' % Just encode output constraints
            constraints = [constraints, ...
                           control_y >= repmat(y_min, 1, N_p + 2 * T_ini), ...
                           control_y <= repmat(y_max, 1, N_p + 2 * T_ini)];
        case 'f' % All constraints
            constraints = [constraints, ...
                           control_u >= repmat(u_min, 1, N_p + 2 * T_ini), ...
                           control_u <= repmat(u_max, 1, N_p + 2 * T_ini), ...
                           control_y >= repmat(y_min, 1, N_p + 2 * T_ini), ...
                           control_y <= repmat(y_max, 1, N_p + 2 * T_ini)];
    end

    %% Define solver settings and run optimization
    switch opt_params.solver_type
        case 'q'
            options = sdpsettings('verbose', 1, 'solver', 'quadprog');
        case 'f'
            options = sdpsettings('verbose', 1, 'solver', 'fmincon');
        case 'o'
            options = sdpsettings('solver', 'OSQP', ...
                  'verbose', 1, ...             % Detailed solver output
                  'osqp.max_iter', 30000, ...   % Set maximum iterations
                  'osqp.eps_abs', 1e-7, ...     % Absolute tolerance
                  'warmstart', 0);             % Disable warm start
    end


    diagnostics = optimize(constraints, objective, options);
        
    if diagnostics.problem == 0 % Feasible solution found
        % Extract optimal values
        u_opt = value(control_u);
        y_opt = value(control_y);
    else
        disp(diagnostics.problem); % Solver exit code
        disp(diagnostics.info);    % Detailed solver feedback        
    end

    if opt_params.verbose
        disp('---- Debug: Objective Function Evaluation ----');
        disp('Objective value:');
        disp(value(objective)); % Ensure it computes as expected
        disp('---- Debug: Feasibility Check ----');
        disp('Max constraint violation:');
        disp(max(check(constraints))); % Show largest constraint violation
    end

end
