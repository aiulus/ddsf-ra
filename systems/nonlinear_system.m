function sys = nonlinear_system(system_type)
    %  INPUT:
    %  system_description [string] - Type of system (e.g., 'cruise_control')
    %  OUTPUT:
    %  sys [struct] - Struct with fields A,B,C,D, constraints, parameters

    sys = struct();

    switch system_type
        case 'inverted_pendulum' % Inverted Pendulum on a Cart
            sys.name = 'Inverted Pendulum';

            % Parameters
            M = 0.5; % mass of the cart [kg]
            m = 0.2; % mass of the pendulum [kg]
            l = 0.3; % length of the pendulum [m]
            g = 9.81; % acceleration due to gravity [m/s^2]
            dt = 0.01; % discretization time step

            % Store parameters in struct for easy access
            sys.parameters.cart_mass = M;
            sys.parameters.pendulum_mass = m;
            sys.parameters.length = l;
            sys.parameters.gravity = g;
            sys.parameters.time_step = dt;

            %% Continuous-Time State-Space Matrices
            A = [0, 1, 0, 0;
                 0, 0, -m * g / M, 0;
                 0, 0, 0, 1;
                 0, 0, (M + m) * g / (M * l), 0];
            B = [0;
                 1 / M;
                 0;
                 -1 / (M * l)];
            C = [1, 0, 0, 0;
                 0, 0, 1, 0];
            D = [0; 0];

            % Discretize the system using the time step `dt`
            sys_cont = ss(A, B, C, D);
            sys_disc = c2d(sys_cont, dt);
            [Ad, Bd, Cd, Dd] = ssdata(sys_disc);

            % System Matrices and Dynamics
            sys.model.A = Ad;
            sys.model.B = Bd;
            sys.model.C = Cd;
            sys.model.D = Dd;

            % Define function handles for dynamics and output
            sys.model.f = @(x, u) sys.model.A * x + sys.model.B * u; % Discrete-time dynamics
            sys.model.h = @(x, u) sys.model.C * x + sys.model.D * u; % Output equation
            
            % Constraints (optional)
            % Define input and state constraints if applicable
            sys.constraints.U = [-10, 10]; % Example input force limits 
            
        case 'quadrotor'
            % Quadrotor 6-DOF system
            sys.name = '6dof-quadrotor';

            % Parameters
            mass = 0.2; % [kg]
            g = 9.81; % [m/sec^2]
            T_s = 0.1; % [sec] sampling time
            I = repmat(10^(-3), 3, 1); % Moment of inertia in x, y, z
            
            % Define parameter struct for readability
            sys.parameters.mass = mass;
            sys.parameters.gravity = g;
            sys.parameters.inertia = I;
            sys.parameters.sampling_time = T_s;

            %% Constraints
            % Define control and state constraints
            sys.constraints.U = [-1, 1; -0.1, 0.1; -0.1, 0.1; -0.1, 0.1]; % Control input constraints
            sys.constraints.theta = [-0.2, 0.2]; % [rad]
            sys.constraints.phi = [-0.2, 0.2];   % [rad]
            sys.constraints.psi = [-0.2, 0.2];   % [rad]
            sys.constraints.position = [-1, 1; -1, 1; -1, 1]; % Position bounds for x, y, z
            
            %% State-space Matrices
            % System dimensions
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            ny = 6;  % Number of outputs

            % Define state-space matrices as sparse for efficiency
            A_i = [1, 2, 3, 10, 11, 12];
            A_j = [4, 5, 6, 7, 8, 9];
            A_val = ones(6, 1);
            A = sparse(A_i, A_j, A_val, nx, nx);
            sys.model.A = A;

            B_i = [9, 4, 5, 6];
            B_j = [1, 2, 3, 4];
            B_val = [1/mass, 1/I(1), 1/I(2), 1/I(3)];
            B = sparse(B_i, B_j, B_val, nx, nu);
            sys.model.B = B;

            % Output matrices (position and orientation tracking)
            C = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            D = 0;

            sys.model.C = C;
            sys.model.D = D;

            %% Dynamics and Output Functions
            % Define function handles for state evolution and output
            sys.model.f = @(x, u) sys.model.A * x + sys.model.B * u;  % Dynamics function
            sys.model.h = @(x, u) sys.model.C * x + sys.model.D * u;  % Measurement/output function
            
        otherwise
            % Error for unrecognized system type
            error('System type "%s" not recognized. Please choose a valid system type.', system_type);
    end
end