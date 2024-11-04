function sys = linear_system(system_description)
    %  INPUT:
    %  system_description [string] - Type of system (e.g., 'cruise_control')
    %  OUTPUT:
    %  sys [struct] - Struct with fields A,B,C,D, constraints, parameters

    sys = struct();

    switch system_description
        case 'cruise_control' % Speed-fixing cruise control
            % Parameters
            m = 1000; % Vehicle mass [kg]
            b = 50; % Damping coefficient [N*s/m]
            dt = 0.1; % Time step for discretization
            u_min = -1000; % Minimum force
            u_max = 1000; % Maximum force

            % State-Space Matrices
            A = 1 - (b * dt) / m;
            B = dt / m;
            C = 1;
            D = 0;

            sys.parameters.vehicle_mass = m; % Vehicle mass [kg]
            sys.parameters.road_friction = b; % Damping coefficient [N*s/m]
            sys.parameters.dt = dt; % Time step for discretization
            
            sys.A = A;
            sys.B = B;
            sys.C = C;
            sys.D = D;

            % Constraints
            sys.constraints.U = [u_min, u_max]; % Force limits for the car

        case 'acc' % Adaptive Cruise Control (ACC) with time delay
           % Parameters
            m_c = 1650; % Mass of the follower car [kg]
            T_s = 0.2; % Sampling time [s]
            delay_steps = 3;
            u_min = -2000; % Minimum control input [N]
            u_max = 2000; % Maximum control input [N]
            N_p = 15; % Prediction horizon
            
            % State-space matrices
            A = [0 1;
            0 0];
            B = [0; 1/m_c];
            C = [1 0];
            D = 0;

            sys_cont = ss(A, B, C, D);

            % Discretize the system
            sys_disc = c2d(sys_cont, T_s);
            [Ad, Bd, Cd, Dd] = ssdata(sys_disc);

            sys.A = Ad;
            sys.B = Bd;
            sys.C = Cd;
            sys.D = Dd;
            
            % Parameters
            sys.delay = delay_steps;
            sys.parameters.mass = m_c;
            sys.parameters.sampling_time = T_s;

            % Constraints
            sys.constraints = [u_min, u_max];
        otherwise
            error('System type "%s" not recognized.', system_description);
    end
end