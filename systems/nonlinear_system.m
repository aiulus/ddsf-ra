function sys = nonlinear_system(system_type)
    %  INPUT:
    %  system_description [string] - Type of system (e.g., 'cruise_control')
    %  OUTPUT:
    %  sys [struct] - Struct with fields A,B,C,D, constraints, parameters

    switch system_type
        case 'inverted_pendulum' % Inverted Pendulum on a Cart
            % Parameters
            params = struct( ...
                'c_mass', 0.5, ... % Mass of the cart [kg]
                'p_mass', 0.2, ... % Mass of the pendulum [kg]
                'I', 0.006, ... % Mass moment of inertia of the pendulum [kg.m^2]
                'l', 0.3, ... % length of the pendulum [m]
                'g', 9.81, ... % Gravity constant [m/s^2]
                'b', 0.1, ... % Friction [N*s/m]
                'dt', 0.1, ... % Time step for discretization
                'y_min', [-inf,-180], ... % Positional constraint
                'y_max', [inf,180], ... % Positional constraint
                'u_min', -10, ... % Minimum force
                'u_max', 10, ... % Maximum force
                'target', [0.2, 0], ... % Desired output
                'x_ini', [0; 0; 0; 0], ... % Initial state [x, x_dot, theta, theta_dot]
                'p', 2, ... % Output dimension
                'm', 1, ... % Input dimension
                'n', 4, ... % State dimension
                'state_name', {"Linear Position, Linear Velocity, Angular Position, Angular Velocity"}, ...
                'input_name', {"Force"}); % Initial velocity [m/s]

            % DeePC configuration
            deepc_config = struct( ...
                'T', 37, ... % Window length
                'T_ini', 10, ... % Initial trajectory length
                'N', 5, ... % Prediction horizon
                's', 2, ... % Sliding length
                'Q', 100000, ... % Output cost matrix
                'R', 0.1 ... % Control cost matrix
            );
            
            M = params.c_mass;
            m = params.p_mass;
            I = params.I;
            l = params.l;
            b = params.b;
            g = params.g;

            % Compute the state-space matrices

            p = I*(M+m)+M*m*l^2; % denominator for the A and B matrices

            A = [0      1              0           0;
                 0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
                 0      0              0           1;
                 0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
            B = [     0;
                 (I+m*l^2)/p;
                      0;
                    m*l/p];
            C = [1 0 0 0;
                 0 0 1 0];
            D = [0;
                 0];

        case 'ddsf_quadrotor'
            % Parameters
            params = struct( ...
                'mass', 0.2, ... % Quadrotor mass [kg]
                'g', 9.81, ... % Gravity constant
                'dt', 0.1, ... % Time step for discretization
                'u_min', (-1)*[1; 0.1; 0.1; 0.1], ... % Minimum force
                'u_max', [1; 0.1; 0.1; 0.1], ... % Maximum force
                'y_min', (-1)*[1; 1; 1; 0.2; 0.2; 0.2], ... % Output constraints
                'y_max', [1; 1; 1; 0.2; 0.2; 0.2], ...  % Output constraints                          
                'I', repmat(10^(-3), 3, 1), ... % Moment of inertia in x, y, z
                'x_ini', zeros(12, 1), ...
                'target', ones(6, 1), ... % TODO: Current value is just a placeholder
                'p', 6, ... % Output dimension (y € R^p)
                'm', 4, ... % Input dimension (u € R^m)
                'n', 12, ... % State dimension (x € R^n)
                'state_name', {"[linpos, delta-linpos, angpos, delta-angpos]"}, ...
                'input_name', {"Total force"}); % Initial velocity [m/s]

            % DeePC configuration
            deepc_config = struct( ...
                'T', 214, ... % Window length
                'T_ini', 1, ... % Initial trajectory length
                'N', 30, ... % Prediction horizon
                's', 1, ... % Sliding length
                'Q', 1500, ... % Output cost matrix
                'R', 0.1 ... % Control cost matrix
            );

            ddsf_config = struct( ...
                'T', 214, ... % Data length
                'T_ini', 2, ... % Initial trajectory length
                'N_p', 20, ... % Prediction horizon
                's', 1, ... % Sliding length
                'R', 1 ... % Cost matrix
            );

            
            %% State-space Matrices

            % Define state-space matrices as sparse for efficiency
            A_i = [1, 2, 3, 10, 11, 12, 8, 7];
            A_j = [4, 5, 6, 7, 8, 9, 1, 2];
            A_val = [ones(6, 1); params.g; -params.g];
            A = sparse(A_i, A_j, A_val, params.n, params.n);

            B_i = [9, 4, 5, 6];
            B_j = [1, 2, 3, 4];
            B_val = [1/params.mass, 1/params.I(1), 1/params.I(2), 1/params.I(3)];
            B = sparse(B_i, B_j, B_val, params.n, params.m);

            % Output matrices (position and orientation tracking)
            % Define the indices of x that correspond to y
            indices = [1, 2, 3, 10, 11, 12]; % Indices for ϕ, θ, ψ, x, y, z in x
            
            % Create C as a sparse matrix
            C = sparse(1:length(indices), indices, 1, length(indices), 12);

            D = zeros(6, 4);

        case 'deepc_quadrotor'
            % Parameters
            params = struct( ...
                'mass', 0.2, ... % Quadrotor mass [kg]
                'g', 9.81, ... % Gravity constant
                'dt', 0.1, ... % Time step for discretization
                'u_min', [0; 0; 0; 0], ... % Minimum force
                'u_max', [1; 1; 1; 1], ... % Maximum force
                'y_min', -[3; 3; 3; inf; inf; inf], ... % Output constraints
                'y_max', [3; 3; 3; inf; inf; inf], ...  % Output constraints                          
                'I', repmat(10^(-3), 3, 1), ... % Moment of inertia in x, y, z
                'x_ini', zeros(12, 1), ...
                'target', -ones(6, 1), ... % TODO: Current value is just a placeholder
                'p', 6, ... % Output dimension (y € R^p)
                'm', 4, ... % Input dimension (u € R^m)
                'n', 12, ... % State dimension (x € R^n)
                'state_name', {"[linpos, delta-linpos, angpos, delta-angpos]"}, ...
                'input_name', {"Total force"}); % Initial velocity [m/s]

            % DeePC configuration
            deepc_config = struct( ...
                'T', 214, ... % Window length
                'T_ini', 1, ... % Initial trajectory length
                'N', 30, ... % Prediction horizon
                'max_iter', 600, ... % # Simulation steps
                's', 2, ... % Sliding length
                'Q', diag([200, 200, 300, 1, 1, 1]), ... % Output cost matrix
                'R', 1, ... % Control cost matrix
                'lambda_g', 30, ... % Regularization parameter
                'lambda_y', (10^5) ... % Regularization parameter
            );

            %% State-space Matrices

            % Define state-space matrices as sparse for efficiency
            A_i = [1, 2, 3, 10, 11, 12, 8, 7];
            A_j = [4, 5, 6, 7, 8, 9, 1, 2];
            A_val = [ones(6, 1); params.g; -params.g];
            A = sparse(A_i, A_j, A_val, params.n, params.n);

            B_i = [9, 4, 5, 6];
            B_j = [1, 2, 3, 4];
            B_val = [1/params.mass, 1/params.I(1), 1/params.I(2), 1/params.I(3)];
            B = sparse(B_i, B_j, B_val, params.n, params.m);

            % Output matrices (position and orientation tracking)
            % Define the indices of x that correspond to y
            indices = [1, 2, 3, 10, 11, 12]; % Indices for ϕ, θ, ψ, x, y, z in x
            
            % Create C as a sparse matrix
            C = sparse(1:length(indices), indices, 1, length(indices), 12);

            D = zeros(6, 4);

         
        otherwise
            % Error for unrecognized system type
            error('System type "%s" not recognized. Please choose a valid system type.', system_type);
    end

    sys = populate_system_struct(A, B, C, D, params); % Assign the parameters to struct object
    %sys = addEquilibriumStates(sys);

    % Add DeePC configuration if defined
    if exist('deepc_config', 'var')
        sys.deepc_config = deepc_config;
    end

    % Add DDSF configuration if defined
    if exist('ddsf_config', 'var')
        sys.ddsf_config = ddsf_config;
    end
end



