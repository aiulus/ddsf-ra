function sys = nonlinearSysInit(sys_type)
    % Initialize system based on type
    switch sys_type
        %% Logistic Map
        case 'logmap'
            params = struct( ...
                'mu', 1, ...             % Nonlinearity parameter
                'x_ini', 2, ...          % Initial state (scalar for logmap)
                'u_min', -5, ...
                'u_max', 5 ...
            );

            syms x u
            % Dynamics
            f1 = params.mu * x * (1 - x);
            g1 = x;

            Fx = {f1}; % Dynamics functions
            Gx = {g1}; % Measurement functions

            % Use symbolic variables
            statevars = x;
            inputvars = u;

            % Validate parameters
            if params.mu <= 0
                error('Parameter mu must be greater than 0 for the logistic map.');
            end

            % System configuration
            config = struct( ...
                'T', 50, ...              % Time horizon
                'T_ini', 10, ...          % Initial trajectory length
                'N', 15 ...               % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        %% Van der Pol Oscillator
        case 'van_der_pol'
            params = struct( ...
                'mu', 1, ...             % Nonlinearity parameter
                'x_ini', [2; 0], ...     % Initial state
                'u_min', -5, ...
                'u_max', 5 ...
            );

            % Dynamics
            syms x1 x2 u
            f1 = x2;
            f2 = params.mu * (1 - x1^2) * x2 - x1 + u;
            g1 = x1;

            Fx = {f1, f2}; % Dynamics functions
            Gx = {g1};     % Measurement functions

            % Use symbolic variables
            statevars = [x1; x2];
            inputvars = u;

            % System configuration
            config = struct( ...
                'T', 100, ...            % Time horizon
                'T_ini', 20, ...         % Initial trajectory length
                'N', 30 ...              % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        %% Nonlinear Pendulum
        case 'nonlinear_pendulum'
            params = struct( ...
                'g', 9.81, ...           % Gravity
                'l', 1, ...              % Length of pendulum
                'r', 2, ...
                'x_ini', [pi/4; 0], ...  % Initial angle and angular velocity
                'u_min', -2, ...
                'u_max', 2 ...
            );

            % Dynamics
            syms x1 x2 u
            f1 = x2;
            f2 = -(params.g / params.l) * sin(x1) - (params.r) * x2;
            g1 = x1;

            Fx = {f1, f2}; % Dynamics functions
            Gx = {g1};     % Measurement functions

            % Use symbolic variables
            statevars = [x1; x2];
            inputvars = u;

            % System configuration
            config = struct( ...
                'T', 80, ...             % Time horizon
                'T_ini', 10, ...         % Initial trajectory length
                'N', 20 ...              % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        otherwise
            error('Unknown system type specified.');
    end

    % Linearize the system
    [A, B, C, D] = linearize(Fx, Gx, statevars, inputvars);
    sys.A = A; sys.B = B; sys.C = C; sys.D = D;
    sys.config = config; sys.opt_params = opt_params;
end

%% Linearization Function
function [A, B, C, D] = linearize(Fx, Gx, x, u)
    % Compute equilibrium
    [x_e, u_e] = getEquilibrium(Fx, x, u);

    % Debug sizes of x, u, x_e, u_e
    disp('x_e:'); disp(x_e); disp('u_e:'); disp(u_e);
    disp('Size of x_e:'); disp(size(x_e));
    disp('Size of u_e:'); disp(size(u_e));
    disp('Size of x:'); disp(size(x));
    disp('Size of u:'); disp(size(u));

    % Initialize symbolic Jacobians
    A_sym = [];
    B_sym = [];
    for i = 1:length(Fx)
        % Compute Jacobians for each dynamic equation in Fx
        A_sym = [A_sym; jacobian(Fx{i}, x)]; % State Jacobian
        B_sym = [B_sym; jacobian(Fx{i}, u)]; % Input Jacobian
    end
    
    % Measurement Jacobians
    C_sym = [];
    D_sym = [];
    for i = 1:length(Gx)
        % Compute Jacobians for each measurement equation in Gx
        C_sym = [C_sym; jacobian(Gx{i}, x)]; % State Jacobian
        D_sym = [D_sym; jacobian(Gx{i}, u)]; % Input Jacobian
    end

    % Ensure dimensions of substitution match
    if numel([x; u]) ~= numel([x_e; u_e])
        error('Mismatch in dimensions of variables and equilibrium values.');
    end

    % Evaluate Jacobians numerically at equilibrium
    A = double(subs(A_sym, [x; u], [x_e; u_e]));
    B = double(subs(B_sym, [x; u], [x_e; u_e]));
    C = double(subs(C_sym, [x; u], [x_e; u_e]));
    D = double(subs(D_sym, [x; u], [x_e; u_e]));
end



%% Equilibrium Computation
function [x_e, u_e] = getEquilibrium(Fx, x, u)
    % Construct equilibrium equations
    eqns = [];
    for i = 1:length(Fx)
        eqns = [eqns; Fx{i} == 0];
    end

    % Solve for equilibrium
    vars = [x(:); u(:)]; % Combine state and input variables
    sol = solve(eqns, vars, 'Real', true);

    % Check if the solution is empty
    if isempty(sol)
        error('No real equilibrium solution found.');
    end

    % Extract the solution into numeric arrays
    if isstruct(sol)
        % Single solution: Convert structure fields to array
        sol_vals = struct2array(sol); % Extract all variables in order
    elseif iscell(sol)
        % Multiple solutions: Use the first one
        sol_vals = struct2array(sol{1});
    else
        error('Unexpected solution format from solve.');
    end

    % Split into state and input equilibrium values
    num_states = numel(x); % Number of state variables
    num_inputs = numel(u); % Number of input variables
    x_e = double(sol_vals(1:num_states));   % Extract state equilibrium
    u_e = double(sol_vals(num_states+1:num_states+num_inputs)); % Extract input equilibrium

    % Reshape to match the dimensions of symbolic variables
    x_e = reshape(x_e, size(x));
    u_e = reshape(u_e, size(u));
end

