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
         %% Inverted Pendulum on a Cart
        case 'inverted_pendulum'
            params = struct( ...
                'g', 9.81, ...         % Gravity
                'l', 1.0, ...          % Length of pendulum
                'm', 0.5, ...          % Mass of pendulum
                'M', 1.0, ...          % Mass of cart
                'x_ini', [0; pi/6; 0; 0], ... % Initial [cart position; pendulum angle; cart velocity; angular velocity]
                'u_min', -10, ...
                'u_max', 10 ...
            );

            % Dynamics
            syms x theta dx dtheta u
            m = params.m; M = params.M; l = params.l; g = params.g;

            % Equations of motion
            f1 = dx;
            f2 = dtheta;
            f3 = (u + m*l*dtheta^2*sin(theta) - m*g*cos(theta)*sin(theta)) / (M + m - m*cos(theta)^2);
            f4 = (g*sin(theta) - cos(theta)*f3) / l;

            g1 = x; % Cart position (output)

            Fx = {f1, f2, f3, f4}; % Dynamics functions
            Gx = {g1}; % Measurement functions

            statevars = [x; theta; dx; dtheta];
            inputvars = u;

            config = struct( ...
                'T', 50, ...          % Time horizon
                'T_ini', 10, ...      % Initial trajectory length
                'N', 15 ...           % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        %% Nonlinear Cruise Control with Time Delay
        case 'nonlinear_cruise'
            params = struct( ...
                'tau', 2, ...         % Time delay
                'a', 0.5, ...         % Acceleration gain
                'b', 0.1, ...         % Nonlinear damping
                'x_ini', 20, ...      % Initial velocity
                'u_min', -2, ...
                'u_max', 2 ...
            );

            syms v u u_tau
            % Dynamics with time delay
            f1 = v + params.a*u_tau - params.b*v^2;
            g1 = v; % Output velocity

            Fx = {f1}; % Dynamics functions
            Gx = {g1}; % Measurement functions

            statevars = v; % Current velocity
            inputvars = [u; u_tau]; % Control input and delayed input

            config = struct( ...
                'T', 100, ...         % Time horizon
                'T_ini', 20, ...      % Initial trajectory length
                'N', 30 ...           % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        %% Quadcopter with Coupled Nonlinear Dynamics
        case 'quadcopter'
            params = struct( ...
                'g', 9.81, ...        % Gravity
                'm', 0.5, ...         % Mass of quadcopter
                'x_ini', zeros(12,1), ... % Initial state (6 position + 6 velocity states)
                'u_min', -5, ...
                'u_max', 5 ...
            );

            syms x y z phi theta ppsi dx dy dz dphi dtheta dppsi u1 u2 u3 u4
            g = params.g; m = params.m;

            % Dynamics
            f1 = dx; % Velocity in x
            f2 = dy; % Velocity in y
            f3 = dz; % Velocity in z
            f4 = (u1/m) * (cos(phi)*cos(theta)) - g; % z-direction acceleration
            f5 = u2; % Roll dynamics
            f6 = u3; % Pitch dynamics
            f7 = u4; % Yaw dynamics

            Fx = {f1, f2, f3, f4, f5, f6, f7}; % Dynamics functions
            Gx = {x, y, z}; % Position outputs

            statevars = [x; y; z; phi; theta; ppsi; dx; dy; dz; dphi; dtheta; dppsi];
            inputvars = [u1; u2; u3; u4];

            config = struct( ...
                'T', 100, ...         % Time horizon
                'T_ini', 20, ...      % Initial trajectory length
                'N', 25 ...           % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        %% Nonlinear Mass-Spring-Damper
        case 'mass_spring_damper'
            params = struct( ...
                'k', 2, ...           % Spring constant
                'c', 1, ...           % Damping coefficient
                'm', 1, ...           % Mass
                'x_ini', [1; 0], ...  % Initial displacement and velocity
                'u_min', -10, ...
                'u_max', 10 ...
            );

            syms x dx u
            k = params.k; c = params.c; m = params.m;

            % Dynamics
            f1 = dx; % Velocity
            f2 = -(k/m)*x - (c/m)*dx^2 + u/m; % Acceleration
            g1 = x; % Displacement output

            Fx = {f1, f2}; % Dynamics functions
            Gx = {g1}; % Measurement functions

            statevars = [x; dx];
            inputvars = u;

            config = struct( ...
                'T', 60, ...          % Time horizon
                'T_ini', 10, ...      % Initial trajectory length
                'N', 15 ...           % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        otherwise
            error('Unknown system type specified.');
    end

    % Linearize the system
    [A, B, C, D] = linearize(Fx, Gx, statevars, inputvars);
    sys.A = A; sys.B = B; sys.C = C; sys.D = D;
    sys.config = config; sys.opt_params = opt_params;
    sys.eq = populateEquilibriumSet(Fx, statevars, inputvars, C, D);
end

%% Linearization Function
function [A, B, C, D] = linearize(Fx, Gx, x, u)
    % Compute equilibrium
    [x_e, u_e] = getEquilibrium(Fx, x, u);

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

function eq = populateEquilibriumSet(Fx, x, u, C, D)
    [x_e, u_e] = getEquilibrium(Fx, x, u);
    y_e = C * x_e +  D * u_e;
    eq = struct('x_e', x_e, 'u_e', u_e, 'y_e', y_e);
end