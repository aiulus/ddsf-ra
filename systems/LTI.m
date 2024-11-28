function sys = LTI(system_description)
    % LTI - Implements simple LTI systems for testing control algorithms
    %
    % INPUT:
    %   system_description - Type of system [string] (e.g.,
    %   'single_integrator')
    %
    % OUTPUT:
    %   sys - [struct] with fields A,B,C,D, constraints, parameters
    sys = struct();

    switch system_description
        case 'single_integrator'
            params = struct( ...
                'm', 1, ... % Input dimension (Velocity)
                'p', 1, ... % Output dimension (y = x in this case)
                'n', 1, ... % System dimension (Position)
                'dt', 0.1, ... % Sampling time
                'u_min', -5, ... 
                'u_max', 5, ...
                'y_min', -inf, ...
                'y_max', inf, ...
                'x_ini', 0, ...
                'target', 10 ...
                );

            deepc_config = struct( ...
                'T', 41, ... % Window length
                'T_ini', 5, ... % Initial trajectory length
                'N', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'Q', 1, ... % Output cost matrix
                'R', 0.1 ... % Control cost matrix
            );

            A = 1;
            B = 1;
            C = 1;
            D = 0;

        case 'double_integrator'
            params = struct( ...
                'm', 1, ... % Input: {'Acceleration'}
                'p', 1, ... % Output dimension 
                'n', 2, ... % System states: {'Position', 'Velocity'}
                'dt', 0.1, ... % Sampling time
                'u_min', -10, ... 
                'u_max', 10, ...
                'y_min', -50, ...
                'y_max', 50, ...
                'x_ini', [0; 0], ...
                'target', [10; 0] ...
                );

            deepc_config = struct( ...
                'T', 43, ... % Window length
                'T_ini', 5, ... % Initial trajectory length
                'N', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'Q', 1, ... % Output cost matrix
                'R', 0.1 ... % Control cost matrix
            );

            A = [1 1; 0 1];
            B = [0; 1];
            C = [1 0];
            D = 0;
            
        case 'mass_spring_dampler'
            % Parameters        
            params = struct( ...
                'm', 1, ... % Input dimension (Velocity)
                'p', 1, ... % Output dimension (y = x in this case)
                'n', 1, ... % System dimension (Position)
                'dt', 0.1, ... % Sampling time
                'u_min', 0, ... 
                'u_max', 0.0001, ...
                'y_min', -inf, ...
                'y_max', inf, ...
                'x_ini', 0, ...
                'target', [1; 0],...
                'mass', 1, ...
                'spring_constant', 1, ...
                'damping_coeff', 0.2 ...
                );

            deepc_config = struct( ...
                'T', 20, ... % Window length
                'T_ini', 5, ... % Initial trajectory length
                'N', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'Q', 1, ... % Output cost matrix
                'R', 0.1 ... % Control cost matrix
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

        case 'dc_motor'
            % Parameters
            params = struct( ...
                'J' , 0.01, ... % Inertia
                'b', 0.1, ... % Damping coefficient
                'K', 0.01, ... % Motor constant
                'R', 1, ... % Resistance
                'L', 0.5, ... % Inductance
                'm', 1, ... % Input dimension (Velocity)
                'p', 1, ... % Output dimension (y = x in this case)
                'n', 1, ... % System dimension (Position)
                'dt', 0.1, ... % Sampling time
                'u_min', -inf, ... % Voltage limits
                'u_max', inf, ... % Voltage limits
                'y_min', -inf, ... % Speed limits
                'y_max', inf, ... % Speed limits
                'x_ini', [0; 0], ...
                'target', [10; 0] ...
                );

            deepc_config = struct( ...
                'T', 20, ... % Window length
                'T_ini', 5, ... % Initial trajectory length
                'N', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'Q', 1, ... % Output cost matrix
                'R', 0.1 ... % Control cost matrix
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
