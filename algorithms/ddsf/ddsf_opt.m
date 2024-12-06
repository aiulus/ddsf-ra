function [u_opt, y_opt] = ddsf_opt(params, u_l, traj_ini)
    %% Extract parameters
    T_ini = params.T_ini;
    N_p = params.N_p;
    m = params.dims.m;
    p = params.dims.p;
    num_cols = params.dims.hankel_cols;
    R = params.R;
    H = params.H;
    U = params.sys.constraints.U;
    Y = params.sys.constraints.Y;
    U_s = params.sys.S_f.U_s;
    Y_s = params.sys.S_f.Y_s;
    
    %% Define symbolic variables
    alpha = sdpvar(num_cols, 1);
    control_u = sdpvar(m, N_p + 2 * T_ini);
    control_y = sdpvar(p, N_p + 2 * T_ini);
    % traj_p = [control_u; control_y];
    

    %% Flatten the variables 
    u_bar = reshape(control_u.', [], 1);
    y_bar = reshape(control_y.', [], 1);
    traj_p_bar = [u_bar; y_bar];

    u_bar_ini = u_bar(1:(T_ini * m));
    y_bar_ini = y_bar(1:T_ini * p);
    traj_bar_ini = [u_bar_ini; y_bar_ini];

    u_ini = traj_ini(1:m, :);
    y_ini = traj_ini(m+1:end, :);
    u_ini_flat = reshape(u_ini.', [], 1);
    y_ini_flat = reshape(y_ini.', [], 1);
    traj_ini_flat = [u_ini_flat; y_ini_flat];

    % DEBUG Sequence
    disp("---------------- SIZES ----------------");
    fprintf("left: "); disp(size(traj_bar_ini));
    fprintf("right: "); disp(size(traj_ini_flat));

    %% Define the objective function and the constraints
    %objective = (control_u - u_l).' * R * (control_u - u_l);

    % DEBUG variables
    left = (control_u(:, 1) - u_l).';
    right = (control_u(:, 1) - u_l);
    objective = left * R * right; 

    % DEBUG Sequence
    disp("---------------- COST FUNCTION ----------------");
    disp("---------------- SIZES ----------------");
    fprintf("left: "); disp(size(left));
    fprintf("middle "); disp(size(R));
    fprintf("right: "); disp(size(right));
    
    % TODO: Add terminal safe-set constraints
    constraints = [traj_p_bar == H * alpha, ...
                   traj_bar_ini == traj_ini_flat, ...
                   %control_u >= repmat(U(:, 1), 1, N_p + 2 * T_ini), ...
                   control_u >= -inf, ...
                   %control_u <= repmat(U(:, 2), 1, N_p + 2 * T_ini), ...
                   control_u <= inf, ...
                   %control_y >= repmat(Y(:, 1), 1, N_p + 2 * T_ini), ...
                   control_y >= -inf, ...
                   %control_y <= repmat(Y(:, 2), 1, N_p + 2 * T_ini) ...
                   control_y <= inf ...
    ];
    
    %% Add the terminal safe set constraints
    for t = (1:(N_p + 2 * T_ini))
        % Basis vectors
        z_u = sdpvar(size(U_s, 2), N_p + 2 * T_ini); 
        z_y = sdpvar(size(Y_s, 2), N_p + 2 * T_ini); 
        constraints = [constraints, control_u(t) == U_s * z_u];
        constraints = [constraints, control_y(t) == Y_s * z_y];
    end

    %% Define solver settings and run optimization
    options_quadprog = sdpsettings('verbose', 0, 'solver', 'quadprog');  
    diagnostics = optimize(constraints, objective, options_quadprog);
        
    if diagnostics.problem == 0 % Feasible solution found
        % Extract optimal values
        u_opt = value(control_u);
        y_opt = value(control_y);
    else
        % Quadprog failed, ask user for next solver choice
        disp('Quadprog failed.');
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
            end
    
        elseif lower(user_input) == 'o' % User chose osqp
            disp('Trying with osqp...');
            options_osqp = sdpsettings('solver', 'OSQP', ...
                  'verbose', 0, ...             % Suppress solver output
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
            end
        end
    end

end

