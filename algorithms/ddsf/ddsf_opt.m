function [u_opt, y_opt] = ddsf_opt(lookup, u_l, traj_ini)
    %% Extract parameters
    % Extract lengths and dimensions
    T_ini = lookup.config.T_ini;
    N_p = lookup.config.N_p;
    L = N_p + 2 * T_ini;
    m = lookup.dims.m;
    p = lookup.dims.p;
    num_cols = lookup.dims.hankel_cols;
    
    % Extract matrices
    C = lookup.sys.C;
    D = lookup.sys.D;
    T_u = lookup.sys.equilibrium.U;
    R = lookup.config.R;
    H = lookup.H;
    
    % Extract constraints
    U = lookup.sys.constraints.U;
    Y = lookup.sys.constraints.Y;
    u_min = U(:, 1);
    u_max = U(:, 2);
    y_min = Y(:, 1);
    y_max = Y(:, 2);

    % Extract the initial trajectory and equilibrium states
    u_ini = traj_ini(1:m, :);
    y_ini = traj_ini(m+1:end, :);
    u_eq = lookup.sys.S_f.u_eq;
    y_eq = lookup.sys.S_f.y_eq;

    
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

    % Element-wise constraints on control inputs
    constraints = [constraints, control_u >= repmat(u_min, 1, N_p + 2 * T_ini)];
    constraints = [constraints, control_u <= repmat(u_max, 1, N_p + 2 * T_ini)];
    
    % Element-wise constraints on outputs
    constraints = [constraints, control_y >= repmat(y_min, 1, N_p + 2 * T_ini)];
    constraints = [constraints, control_y <= repmat(y_max, 1, N_p + 2 * T_ini)];

    %% Define solver settings and run optimization
    options_quadprog = sdpsettings('verbose', 1, 'solver', 'quadprog');  
    diagnostics = optimize(constraints, objective, options_quadprog);
        
    if diagnostics.problem == 0 % Feasible solution found
        % Extract optimal values
        u_opt = value(control_u);
        y_opt = value(control_y);
    else
        % Quadprog failed, ask user for next solver choice
        disp('Quadprog failed.'); 
        disp(diagnostics.problem); % Solver exit code
        disp(diagnostics.info);    % Detailed solver feedback

        user_input = '';
        while ~ismember(lower(user_input), {'f', 'o'})
            user_input = input('Enter "f" for fmincon or "o" for osqp: ', 's');
        end
    
        if lower(user_input) == 'f' % User chose fmincon
            disp('Trying with fmincon...');
            options_fmincon = sdpsettings('verbose', 1, 'solver', 'fmincon');
            diagnostics = optimize(constraints, objective, options_fmincon);
    
            if diagnostics.problem == 0 % Feasible solution found with fmincon
                % Extract optimal values
                u_opt = value(control_u);
                y_opt = value(control_y);
            else
                error('Optimization problem is infeasible even with fmincon!');
                disp(diagnostics.problem); % Solver exit code
                disp(diagnostics.info);    % Detailed solver feedback
            end
    
        elseif lower(user_input) == 'o' % User chose osqp
            disp('Trying with osqp...');
            options_osqp = sdpsettings('solver', 'OSQP', ...
                  'verbose', 1, ...             % Detailed solver output
                  'osqp.max_iter', 30000, ...   % Set maximum iterations
                  'osqp.eps_abs', 1e-7, ...     % Absolute tolerance
                  'warmstart', 0);             % Disable warm start
    
            diagnostics = optimize(constraints, objective, options_osqp);
    
            if diagnostics.problem == 0 % Feasible solution found with osqp
                % Extract optimal values
                u_opt = value(control_u);
                y_opt = value(control_y);
            else
                error('Optimization problem is infeasible even with osqp!');
                disp(diagnostics.problem); % Solver exit code
                disp(diagnostics.info);    % Detailed solver feedback
            end
        end
        disp('---- Debug: Objective Function Evaluation ----');
        disp('Objective value:');
        disp(value(objective)); % Ensure it computes as expected

        disp('---- Debug: Feasibility Check ----');
        disp('Max constraint violation:');
        disp(max(check(constraints))); % Show largest constraint violation
    end

end

