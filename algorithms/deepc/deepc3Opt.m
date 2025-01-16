function [u, y] = deepc3Opt(lookup, H, u_ini, y_ini)
    verbose = false; % Toggle debug mode
    optimizer_type = 'o'; % Toggle optimization type 
    constr_type = 'f'; % Toggle constraint type

    %% Extract DeePC parameters
    % TODO: Line 8 gets interpreted as a function call by the b&b solver
    Q = lookup.deepc.Q;
    R = lookup.deepc.R;
    lambda_g = lookup.deepc.lambda_g;
    
    % Extract dimensions
    m = lookup.dims.m;
    p = lookup.dims.p;
    
    % Extract configuration parameters
    T = lookup.config.T;
    T_ini = lookup.config.T_ini;
    N = lookup.config.N;

    % Extract (past/future slices of) Hankel matrices
    Up = H.Up; Yp = H.Yp;
    Uf = H.Uf; Yf = H.Yf;

    % Extract system constraints
    U = lookup.sys.constraints.U;
    Y = lookup.sys.constraints.Y;
    target = lookup.sys.params.target;

    %% Define symbolic variables
    g = sdpvar(T - T_ini - N + 1, 1);
    u = sdpvar(N * m, 1);
    y = sdpvar(N * p, 1);


    %% Construct the reference trajectory
    target = reshape(repmat(target, N, 1).', [], 1);
    % nancols = isnan(target);
    numcols = ~isnan(target);    

    %% Define the optimization objective
    delta = y(numcols) - target(numcols);
    Qext = kron(eye(N), Q);
    Qext_cut = Qext(numcols, numcols);
    
    objective = delta' * Qext_cut * delta + u' * kron(eye(N), R) * u;

    %% TODO
    %regularized_objective = g' * (lambda_g *  eye(length(g))) * g ...
    %            + (r - y)' * kron(eye(N), Q) * (r - y) + ...
    %            u' * kron(eye(N), R) * u;
    
    u_lb = reshape(repmat(U(:, 1), N, 1).', [], 1);
    u_ub = reshape(repmat(U(:, 2), N, 1).', [], 1);
    y_lb = reshape(repmat(Y(:, 1), N, 1).', [], 1);
    y_ub = reshape(repmat(Y(:, 2), N, 1).', [], 1);

    % Define the constraints
    if lower(constr_type) == 'f'           % Full 
        constraints = [Up * g == u_ini, ...
                       Yp * g == y_ini, ...
                       Uf * g == u, ...
                       Yf * g == y, ...
                       u_lb <= u & u <= u_ub, ...
                       y_lb <= y & y <= y_ub ...
                      ];
    elseif lower(constr_type) == 'r'       % Regularized
        Ux = [Up; Uf]; Yx = [Yp; Yf];
        order = size(Ux, 1);
        Ux = svdHankel(Ux, order);
        Yx = svdHankel(Yx, order); 
        gr = sdpvar(order, 1);
        objective = objective + gr' * (lambda_g * eye(length(gr))) * gr;
        constraints = [[Ux; Yx] * gr == [u_ini; u; y_ini; y], ...
                       u_lb <= u & u <= u_ub, ...
                       y_lb <= y & y <= y_ub ...
                      ];
    elseif lower(constr_type) == 's'       % Simple, just models the system 
        constraints = [Up * g == u_ini, ...% dynamics - same as setting
                       Yp * g == y_ini, ...% all input constraints to
                       Uf * g == u, ...    % +/-inf
                       Yf * g == y
                       ];
    elseif lower(constr_type) == 'e'       % Empty - meant to be used just
        constraints = [];                  % as sanity check 
    end

    if lower(optimizer_type) == 'q'
        options = sdpsettings('solver', 'quadprog', 'verbose', verbose);
    elseif lower(optimizer_type) == 'f'
        options = sdpsettings('solver', 'fmincon', ...
                              'sdpa.maxIteration', 10, ...
                              'verbose', 0);                             
        %options = sdpsettings('solver', 'fmincon', ...
        %                    'relax', 0, ...
        %                    'warning', 0, ...
        %                    'showprogress', 1 ...
        %                    );
    elseif lower(optimizer_type) == 'o'
        options = sdpsettings('solver', 'OSQP', ... % Use OSQP solver
                      'verbose', verbose, ...             
                      'osqp.max_iter', 30000, ...   % Set maximum iterations
                      'osqp.eps_abs', 1e-5, ...     % Absolute tolerance
                      'warmstart', 0);             % Disable warm start
    elseif lower(optimizer_type) == 'b'
        options = sdpsettings('solver', 'bmibnb', 'verbose', 1);
    else
        error('Error assigning solver options!');
    end
    
    diagnostics = optimize(constraints, objective, options);

    if diagnostics.problem == 0
                % g = value(g);
                u = value(u);
                y = value(y);
        if verbose
            disp("REFERENCE TRAJECTORY: "); disp(target');
            disp("DELTA: "); disp(value(delta)');
            disp("PEANALTY TERM: "); disp(value(objective));
        end
    else
        disp(diagnostics.info);
        error('The problem did not solve successfully.');
    end
end

%   CRUISE CONTROL - DeePC
%
%   best_params = 
%
%  struct with fields:
%
%       Q: 10000
%        R: 0.0100
%    T_ini: 10
%       N: 30
%
%   in 1348/60