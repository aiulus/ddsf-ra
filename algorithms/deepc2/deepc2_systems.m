function sys = deepc2_systems(sys_type)
    switch sys_type
        %% Case 1: Example0
        case 'example0'
            params = struct( ...
                'target', [0, 0, 0] ...
                );

            sys = struct( ...
                'A', [1 -0.01 0; ...
                      0.01 1 0;
                      0 0 0.8], ...  
                'B', eye(3,2), ...
                'C', eye(3), ...
                'D', zeros(3, 2) ...
             );

            run_config = struct( ...
                'T', 138, ... % Window length
                'T_ini', 5, ... % Initial trajectory length
                'T_f', 4, ... % Prediction horizon
                's', 2 ... % Sliding length
            );

            opt_params = struct( ...
                'Q', eye(size(sys.C, 1)), ... % Output cost matrix
                'R', eye(size(sys.B, 2)) ... % Control cost matrix
                 );
        
        %% Case 2: Cruise Control
        case 'cruise_control'
            % System-specific parameters
            params = struct( ...
                'mass', 1000, ... % Vehicle mass [kg]
                'damping', 50, ... % Damping coefficient [N*s/m]
                'dt', 0.1, ... % Sampling rate for discetization [s]
                'u_min', -inf, ... % Minimum force
                'u_max', inf, ... % Maximum force
                'y_min', -inf, ... % Output constraint
                'y_max', inf, ... % Output constraint
                'target', 20, ... % Reference velocity [m/s]
                'slack', 1e-2, ... % For relaxation  
                'x_ini', 0, ...
                'state_name', {"Velocity"}, ...
                'input_name', {"Force"}); % Initial velocity [m/s]

            % 'target', [1, 2, 3, 4] for dims.p = 4
            %  The constraint then just refers to the decision variable at
            %   y(index) being near target(index)

            A = 1 - (params.damping * params.dt) / params.mass;
            B = params.dt / params.mass;
            C = 1;
            D = 0;

            sys = struct( ...
                'A', A, ...
                'B', B, ...
                'C', C, ...
                'D', D ...
                );

            run_config = struct( ...
                'T', 41, ... % Window length
                'T_ini', 5, ... % Initial trajectory length
                'T_f', 15, ... % Prediction horizon
                's', 3 ... % Sliding length
            );

            opt_params = struct( ...
                        'Q', 100000 * eye(size(sys.C, 1)), ... % Output cost matrix 
                        'R', 0.1 * eye(size(sys.B, 2)) ... % Input cost matrix 
                         ); % Optimization parameters
        
        %% Case 3: Inverted Pendulum
        case 'inverted_pendulum'
            params = struct( ...
                'c_mass', 0.5, ... % Mass of the cart [kg]
                'p_mass', 0.2, ... % Mass of the pendulum [kg]
                'I', 0.006, ... % Mass moment of inertia of the pendulum [kg.m^2]
                'l', 0.3, ... % length of the pendulum [m]
                'g', 9.81, ... % Gravity constant [m/s^2]
                'b', 0.1, ... % Friction [N*s/m]
                'dt', 0.1, ... % Time step for discretization
                'y_min', [-inf;-180], ... % Positional constraint
                'y_max', [inf;180], ... % Positional constraint
                'u_min', -10, ... % Minimum force
                'u_max', 10, ... % Maximum force
                'target', [0.15; 90], ... % Desired output
                'x_ini', [0; 0; 0; 0], ... % Initial state [x, x_dot, theta, theta_dot]
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

            sys = struct( ...
                'A', A, ...
                'B', B, ...
                'C', C, ...
                'D', D ...
                );

            opt_params = struct( ...
                'Q', 1 * eye(size(sys.C, 1)), ... % Output cost matrix 
                'R', 1 * eye(size(sys.B, 2)) ... % Input cost matrix 
             ); % Optimization parameters
            
            run_config = struct( ...
                'T', 37, ... % Window length
                'T_ini', 10, ... % Initial trajectory length
                'T_f', 5, ... % Prediction horizon
                's', 2 ... % Sliding length
            );
    end
  
    sys = deepc2_populate_system(sys, params, opt_params, run_config);
end

function sys = deepc2_populate_system(sys, params, opt_params, run_config)
    % Assign system-specific parameters
    sys.params = params;

    % Assign dimensions
    dims = struct( ...
        'n', size(sys.A, 1), ... % System state dim.
        'm', size(sys.B, 2), ... % Input dim.
        'p', size(sys.C, 1), ... % Output shape
        'q', size(sys.B, 2) + size(sys.C, 1) ... % #I/O variables
    );

    sys.dims = dims;
    largeval = 1e+30;

    % Assign constraints
    if isfield(params, 'u_min')
        if max(size(params.u_min)) == 1
            u_min = repmat(params.u_min, dims.m, 1);
        else
            u_min = params.u_min;
        end
    else
        u_min = repmat(-largeval, dims.m, 1);
    end

    if isfield(params, 'u_max')
        if max(size(params.u_max)) == 1
            u_max = repmat(params.u_max, dims.m, 1);
        else
            u_max = params.u_max;
        end
    else
        u_max = repmat(largeval, dims.m, 1);
    end

    if isfield(params, 'y_min')
        if max(size(params.y_min)) == 1
            y_min = repmat(params.y_min, dims.p, 1);
        else
            y_min = params.y_min;
        end
    else
        y_min = repmat(-largeval, dims.p, 1);
    end

    if isfield(params, 'y_max')
        if max(size(params.y_max)) == 1
            y_max = repmat(params.y_max, dims.p, 1);
        else
            y_max = params.y_max;
        end
    else
        y_max = repmat(largeval, dims.p, 1);
    end

    sys.constraints.U = [u_min, u_max];
    sys.constraints.Y = [y_min, y_max];
    
    % Assign config. & optimization parameters
    sys.run_config = run_config;
    sys.opt_params = opt_params;
end

