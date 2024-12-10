function sys = deepc2_systems(sys_type)
    switch sys_type
        case 'example0'
            params = struct( ...
                'target', [0; 0; 0] ...
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
                's', 2 ... % Sliding length
            );

            opt_params = struct( ...
                        'Q', 150000 * eye(size(sys.C, 1)), ... % Output cost matrix 
                        'R', 0.1 * eye(size(sys.B, 2)) ... % Input cost matrix 
                         ); % Optimization parameters
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
        end
    else
        u_min = repmat(-largeval, dims.m, 1);
    end

    if isfield(params, 'u_max')
        if max(size(params.u_max)) == 1
            u_max = repmat(params.u_max, dims.m, 1);
        end
    else
        u_max = repmat(largeval, dims.m, 1);
    end

    if isfield(params, 'y_min')
        if max(size(params.y_min)) == 1
            y_min = repmat(params.y_min, dims.p, 1);
        end
    else
        y_min = repmat(-largeval, dims.p, 1);
    end

    if isfield(params, 'y_max')
        if max(size(params.y_max)) == 1
            y_max = repmat(params.y_max, dims.p, 1);
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

