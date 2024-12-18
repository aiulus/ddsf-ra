function sys = ddsf_systems(sys_type, discretize)
    switch sys_type
        %% Example 1: The Quadrotor
        case 'ddsf_quadrotor'
            % System-specific parameters
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
                'target', ones(6, 1) ... % TODO: Current value is just a placeholder
                );

            run_config = struct( ...
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

        %% Example 2: Mass Spring Dampler
        case 'dampler'
            params = struct( ...
               'dt', 0.1, ... % Sampling time
                'u_min', -100, ... 
                'u_max', 100, ...
                'y_min', -inf, ...
                'y_max', inf, ...
                'x_ini', [0.5;0.5], ... % y_ini = x_ini(1)
                'target', 5,...
                'mass', 1, ...
                'spring_constant', 1, ...
                'damping_coeff', 0.2 ...
                );

            dt = params.dt;
            m = params.mass;
            b = params.damping_coeff;
            k = params.spring_constant;

            % State-space matrices
            A = [1 dt; -k/m*dt 1 - b/m*dt];
            B = [0; dt/m];
            C = [1 0];
            D = 0;    

            run_config = struct( ...
                'T', 25, ... % Data length
                'T_ini', 5, ... % Initial trajectory length
                'N_p', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'R', 1 ... % Cost matrix
            );

        %% Example 3: Inverted Pendulum
        case 'inverted_pendulum'
            params = struct( ...
                'c_mass', 50, ... % Mass of the cart [kg]
                'p_mass', 2, ... % Mass of the pendulum [kg]
                'I', 0.6, ... % Mass moment of inertia of the pendulum [kg.m^2]
                'l', 3, ... % length of the pendulum [m]
                'g', 9.81, ... % Gravity constant [m/s^2]
                'b', 0.1, ... % Friction [N*s/m]
                'dt', 0.1, ... % Time step for discretization
                'y_min', [0;-inf], ... % Positional constraint
                'y_max', [1.5;inf], ... % Positional constraint
                'u_min', -inf, ... % Minimum force
                'u_max', inf, ... % Maximum force
                'target', [1.45, NaN], ... % Desired output
                'x_ini', [0.5; 0; 0; 0], ... % Initial state [x, x_dot, theta, theta_dot]
                'state_name', {"Linear Position, Linear Velocity, Angular Position, Angular Velocity"}, ...
                'input_name', {"Force"}); % Initial velocity [m/s]

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

            run_config = struct( ...
                'T', 25, ... % Data length
                'T_ini', 5, ... % Initial trajectory length
                'N_p', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'R', 1 ... % Cost matrix
            );

        %% Example 4: DC Motor
        case 'dc_motor'
            params = struct( ...
                'J' , 0.01, ... % Inertia
                'b', 0.1, ... % Damping coefficient
                'K', 0.01, ... % Motor constant
                'R', 1, ... % Resistance
                'L', 0.5, ... % Inductance
                'dt', 0.1, ... % Sampling time
                'u_min', -inf, ... % Voltage limits
                'u_max', inf, ... % Voltage limits
                'y_min', -inf, ... % Speed limits
                'y_max', inf, ... % Speed limits
                'x_ini', [1; 1], ... % y_ini = x_ini(1)
                'target', 10 ...
                );
                        
            b = params.b;
            J = params.J;
            K = params.K;
            R = params.R;
            L = params.L;
            
            A = [-b/J K/J; -K/L -R/L];
            B = [0; 1/L];
            C = [1 0];
            D = 0;

            run_config = struct( ...
                'T', 25, ... % Data length
                'T_ini', 5, ... % Initial trajectory length
                'N_p', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'R', 1 ... % Cost matrix
            );

        %% Example 5: Cruise Control
        case 'cruise_control'
            % System-specific parameters
            params = struct( ...
                'mass', 1000, ... % Vehicle mass [kg]
                'damping', 50, ... % Damping coefficient [N*s/m]
                'dt', 0.1, ... % Sampling rate for discetization [s]
                'u_min', 0, ... % Minimum force
                'u_max', 5000, ... % Maximum force
                'y_min', -inf, ... % Output constraint
                'y_max', inf, ... % Output constraint
                'target', 10, ... % Reference velocity [m/s]
                'slack', 1e-2, ... % For relaxation  
                'x_ini', 20, ...
                'state_name', {"Velocity"}, ...
                'input_name', {"Force"}); % Initial velocity [m/s]

            A = 1 - (params.damping * params.dt) / params.mass;
            B = params.dt / params.mass;
            C = 1;
            D = 0;
            
            run_config = struct( ...
                'T', 25, ... % Data length
                'T_ini', 1, ... % Initial trajectory length
                'N_p', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'R', 1 ... % Cost matrix
            );
    end

    if discretize == true
        [A, B, C, D] = discretize_system(A, B, C, D, params.dt);
    end
    % Collect all system properties in a single object
    sys = populate_system_struct(A, B, C, D, params);
    % Parse constraints
    sys = constraint_handler(sys, params);
    sys.config = run_config;
end

% TODO: Remove -currently not in use
%function sys = ddsf_discretize(sys)
%    ssc = ss(sys.A, sys.B, sys.C, sys.D);
%    ssd = c2d(ssc, sys.params.dt);

%    sys = struct( ...
%        'A', ssd.A, ...
%        'B', ssd.B, ...
%        'C', ssd.C, ...
%        'D', ssd.D ...
%    );
%end

