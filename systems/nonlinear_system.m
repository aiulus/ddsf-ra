function sys = nonlinear_system(system_type)
    %  INPUT:
    %  system_description [string] - Type of system (e.g., 'cruise_control')
    %  OUTPUT:
    %  sys [struct] - Struct with fields A,B,C,D, constraints, parameters

    switch system_type
        case 'inverted_pendulum' % Inverted Pendulum on a Cart
            x_ini = [0; 0; 0; 0];  % [x, x_dot, theta, theta_dot]
            % Parameters
            params = struct( ...
                'c_mass', 0.5, ... % Mass of the cart [kg]
                'p_mass', 0.2, ... % Mass of the pendulum [kg]
                'I', 0.006, ... % Mass moment of inertia of the pendulum [kg.m^2]
                'l', 0.3, ... % length of the pendulum [m]
                'g', 9.81, ... % Gravity constant [m/s^2]
                'b', 0.1, ... % Friction [N*s/m]
                'dt', 0.1, ... % Time step for discretization
                'u_min', -10, ... % Minimum force
                'u_max', 10, ... % Maximum force
                'target', -10, ... % Reference velocity [m/s]
                'x_ini', x_ini, ... % [x, x_dot, theta, theta_dot]
                'p', 2, ... % Output dimension
                'm', 1, ... % Input dimension
                'n', 4, ... % State dimension
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

        case 'quadrotor'
            % Parameters
            params = struct( ...
                'mass', 0.2, ... % Quadrotor mass [kg]
                'g', 9.81, ... % Gravity constant
                'dt', 0.1, ... % Time step for discretization
                'u_min', -[1; 0.1; 0.1; 0.1], ... % Minimum force
                'u_max', [1; 0.1; 0.1; 0.1], ... % Maximum force
                'y_min', -[1; 1; 1; inf; inf; inf; % Output constraints
                          0.2; 0.2; 0.2; inf; inf; inf], ...
                'y_max', [1; 1; 1; inf; inf; inf; % Output constraints
                          0.2; 0.2; 0.2; inf; inf; inf], ...
                'I', repmat(10^(-3), 3, 1), ... % Moment of inertia in x, y, z
                'x_ini', zeros(12, 1), ...
                'target', zeros(6, 1), ... % TODO: Current value is just a placeholder
                'p', 6, ... % Output dimension (y € R^p)
                'm', 4, ... % Input dimension (u € R^m)
                'n', 12, ... % State dimension (x € R^n)
                'state_name', {"[linpos, delta-linpos, angpos, delta-angpos]"}, ...
                'input_name', {"Total force"}); % Initial velocity [m/s]

            
            %% State-space Matrices

            % Define state-space matrices as sparse for efficiency
            A_i = [1, 2, 3, 10, 11, 12];
            A_j = [4, 5, 6, 7, 8, 9];
            A_val = ones(6, 1);
            A = sparse(A_i, A_j, A_val, params.n, params.n);

            B_i = [9, 4, 5, 6];
            B_j = [1, 2, 3, 4];
            B_val = [1/params.mass, 1/params.I(1), 1/params.I(2), 1/params.I(3)];
            B = sparse(B_i, B_j, B_val, params.n, params.m);

            % Output matrices (position and orientation tracking)
            C = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            D = 0;

            
        otherwise
            % Error for unrecognized system type
            error('System type "%s" not recognized. Please choose a valid system type.', system_type);
    end
    sys = populate_system_struct(A, B, C, D, params); % Assign the parameters to struct object
end


%% Utility Functions
function [Ad, Bd, Cd, Dd] = discretize_system(A, B, C, D, dt)
    % Discretizes a continuous-time state-space system.
    sys_cont = ss(A, B, C, D);
    sys_disc = c2d(sys_cont, dt);
    [Ad, Bd, Cd, Dd] = ssdata(sys_disc);
end

function sys = populate_system_struct(A, B, C, D, params)
    % Constructs the system struct with matrices, dimensions, and constraints.
    sys.A = A;
    sys.B = B;
    sys.C = C;
    sys.D = D;

    % Dimensions
    sys.dims.state = size(A, 1);
    sys.dims.input = size(B, 2);
    sys.dims.output = size(C, 1);

    % Constraints
    sys.constraints.U = [params.u_min, params.u_max];
    sys.constraints.Y = [-inf, inf];

    % Parameters and target
    sys.params = params;
    sys.target = params.target;

    [Ad, Bd, Cd, Dd] = discretize_system(A, B, C, D, params.dt);
    sys.Ad = Ad;
    sys.Bd = Bd;
    sys.Cd = Cd;
    sys.Dd = Dd;
end