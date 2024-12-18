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

