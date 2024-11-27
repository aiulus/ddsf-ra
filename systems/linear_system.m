function sys = linear_system(system_description)
    % LINEAR_SYSTEM Encodes state-space LTI systems with parameters.
    %
    % INPUT:
    %    system_description [string] - Type of system (e.g., 'cruise_control')
    % OUTPUT:
    %    sys [struct] - Struct with fields A, B, C, D, constraints, parameters.

    switch system_description
        case 'cruise_control'
            % Parameters
            params = struct( ...
                'mass', 1000, ... % Vehicle mass [kg]
                'damping', 50, ... % Damping coefficient [N*s/m]
                'dt', 0.1, ... % Time step for discretization
                'u_min', -inf, ... % Minimum force
                'u_max', inf, ... % Maximum force
                'y_min', -inf, ... % Output constraint
                'y_max', inf, ... % Output constraint
                'target', 20, ... % Reference velocity [m/s]
                'x_ini', 0, ...
                'p', 1, ... % Output dimension
                'm', 1, ... % Input dimension
                'n', 1, ... % State dimension
                'state_name', {"Velocity"}, ...
                'input_name', {"Force"}); % Initial velocity [m/s]

            % DeePC configuration
            deepc_config = struct( ...
                'T', 41, ... % Window length
                'T_ini', 5, ... % Initial trajectory length
                'N', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'Q', 150000, ... % Output cost matrix
                'R', 0.1 ... % Control cost matrix
            );
            
            % State-space matrices
            A = 1 - (params.damping * params.dt) / params.mass;
            B = params.dt / params.mass;
            C = 1;
            D = 0;

        case 'simple_integrator'
            % Parameters
            params = struct( ...
                'u_min', -1, ... % Minimum velocity
                'u_max', 1, ... % Maximum velocity
                'y_min', -inf, ... % Output constraint
                'y_max', inf, ... % Output constraint
                'target', 10, ... % Reference position
                'dt', 0.1 ... % Time step for discretization
                 ); 
            
            % State-space matrices
            A = 1;
            B = 1;
            C = 1;
            D = 0;

        case 'acc' % Adaptive Cruise Control (ACC) with time delay
            % Parameters
            params = struct( ...
                'mass', 1650, ... % Follower car mass [kg]
                'sampling_time', 0.2, ... % Sampling time [s]
                'delay_steps', 3, ... % Delay in steps
                'u_min', -2000, ... % Minimum control input [N]
                'u_max', 2000, ... % Maximum control input [N]
                'y_min', -inf, ... % Output constraint
                'y_max', inf, ... % Output constraint
                'target', 20); % Reference position
            
            % Continuous-time state-space matrices
            A = [0 1; 0 0];
            B = [0; 1 / params.mass];
            C = [1 0];
            D = 0;

            % Discretize the system
            [A, B, C, D] = discretize_system(A, B, C, D, params.sampling_time);

        otherwise
            error('System type "%s" not recognized.', system_description);
    end

    % Populate system struct
    sys = populate_system_struct(A, B, C, D, params);

    % Add DeePC configuration if defined
    if exist('deepc_config', 'var')
        sys.deepc_config = deepc_config;
    end
end



