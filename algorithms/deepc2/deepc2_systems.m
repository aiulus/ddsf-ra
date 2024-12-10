function sys = deepc2_systems(sys_type)
    switch sys_type
        case 'example0'
            sys = struct( ...
                'A', [1 -0.01 0; ...
                      0.01 1 0;
                      0 0 0.8], ...  
                'B', eye(3,2), ...
                'C', eye(3), ...
                'D', zeros(3, 2) ...
             );
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
                'x_ini', 0, ...
                'state_name', {"Velocity"}, ...
                'input_name', {"Force"}); % Initial velocity [m/s]

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
                'N', 15, ... % Prediction horizon
                's', 2, ... % Sliding length
                'Q', 1, ... % Output cost matrix 150000
                'R', 0.1 ... % Input cost matrix 0.1
            );
            sys.constraints.U = [params.u_min, params.u_max];
            sys.constraints.Y = [params.y_min, params.y_max];
            sys.run_config = run_config;
    end
end

