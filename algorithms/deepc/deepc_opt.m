%% Defines and solves an optimization problem as described in DeePC

function [g_opt, u_opt, y_opt] = deepc_opt(Up, Yp, Uf, Yf, u_ini, y_ini, r, sys)
    Q = sys.deepc_config.Q;
    R = sys.deepc_config.R;
    N = sys.deepc_config.N;
    U = sys.constraints.U;
    Y = sys.constraints.Y;

    g = sdpvar(size(Uf, 2), 1); % Canocical optimization variable € R^{(T - T_{ini} - N + 1) x 1}

    u = Uf * g; % Predicted inputs (Nm x 1)
    y = Yf * g; % Predicted outputs (Np x 1)
    u = reshape(u, [], sys.params.m);
    y = reshape(y, [], sys.params.p);
    
    % Define the cost function
    cost = 0;
    for k = 1:N
        % cost = cost + (y(k) - r(k, :)).' * Q * (y(k) - r(k, :)) + u(k).' * R * u(k);
        cost = cost + (y(k) - r(k, :)) * Q * (y(k) - r(k, :)).' + u(k) * R * u(k).';
    end

    u_ini = reshape(u_ini, [], 1); % Flatten the initial trajectory for
    y_ini = reshape(y_ini, [], 1); % matching dimensions in constraints
    
    % Define the constraints
    constraints = [Up * g == u_ini, Yp * g == y_ini, ...,
               -u <= -U(1) * ones(size(u)), ...
               u <= U(2) * ones(size(u)), ...
               -y <= -Y(1) * ones(size(y)), ...
               y <= Y(2) * ones(size(y))];
    % TODO: Dimensionality correction with '* ones()' potentially
    % problematic for other instances


    % Solve optimization problem using quadprog
    options_quadprog = sdpsettings('verbose', 0, 'solver', 'quadprog');  
    diagnostics = optimize(constraints, cost, options_quadprog);
        
    if diagnostics.problem == 0 % Feasible solution found
        % Extract optimal values
        g_opt = value(g);
        u_opt = value(u);
        y_opt = value(y);
    else
        % Quadprog failed, ask user for next solver choice
        disp(['Quadprog failed.' ...
            ' and <o> to proceed with OSQP.']);
        user_input = '';
        while ~ismember(lower(user_input), {'f', 'o'})
            user_input = input('Enter "f" for fmincon or "o" for osqp: ', 's');
        end
    
        if lower(user_input) == 'f' % User chose fmincon
            disp('Trying with fmincon...');
            options_fmincon = sdpsettings('verbose', 1, 'solver', 'fmincon');
            diagnostics = optimize(constraints, cost, options_fmincon);
    
            if diagnostics.problem == 0 % Feasible solution found with fmincon
                % Extract optimal values
                g_opt = value(g);
                u_opt = value(u);
                y_opt = value(y);
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
    
            diagnostics = optimize(constraints, cost, options_osqp);
    
            if diagnostics.problem == 0 % Feasible solution found with osqp
                % Extract optimal values
                g_opt = value(g);
                u_opt = value(u);
                y_opt = value(y);
            else
                error('Optimization problem is infeasible even with osqp!');
            end
        end
    end
    disp("OPTIMAL VALUE g: "); disp(g_opt);
    disp("OPTIMAL VALUE u: "); disp(u_opt);
    disp("OPTIMAL VALUE y: "); disp(y_opt);
end

 