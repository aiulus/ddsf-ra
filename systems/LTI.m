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
            sys.A = 1;
            sys.B = 1;
            sys.C = 1;
            sys.D = 0;
            
            sys.dims.state = 1; % n - dim(x)
            sys.dims.input = 1; % m - dim(u)
            sys.dims.output = 1; % p - dim(y)

            sys.parameters.state_name = 'Position';
            sys.parameters.input_name = 'Velocity';
            sys.parameters.dt = 0.1; % Sampling time

            sys.constraints.U = [-5,5];
            sys.constraints.Y = [-inf, inf];

            sys.x_ini = 0; % Starting position
            sys.target = 10; % Desired position

        case 'double_integrator'
            sys.A = [1 1; 0 1];
            sys.B = [0; 1];
            sys.C = [1 0];
            sys.D = 0;

            sys.dims.state = size(sys.A, 1); % n - dim(x)
            sys.dims.input = size(sys.B, 2); % m - dim(u)
            sys.dims.output = size(sys.C, 1); % p - dim(y)

            sys.parameters.state_name = {'Position', 'Velocity'};
            sys.parameters.input_name = {'Acceleration'};
            sys.parameters.dt = 0.1; % Sampling time

            sys.constraints.U = [-10,10]; % Acceleration limits
            sys.constraints.Y = [-50, 50]; % Position limits

            sys.x_ini = [0; 0]; % Starting position and velocity
            sys.target = [10; 0]; % Desired position and velocity
            
        case 'mass_spring_dampler'
            m = 1; % Mass
            k = 1; % Spring constant
            b = 0.2; % Damping coefficient
            dt = 0.1; % Sampling time

            % State-space matrices
            sys.A = [1 dt; -k/m*dt 1 - b/m*dt];
            sys.B = [0; dt/m];
            sys.C = [1 0];
            sys.D = 0;    

            sys.dims.state = size(sys.A, 1); % n - dim(x)
            sys.dims.input = size(sys.B, 2); % m - dim(u)
            sys.dims.output = size(sys.C, 1); % p - dim(y)

            sys.parameters.mass = m;
            sys.parameters.spring_constant = k;
            sys.parameters.damping = b;
            sys.parameters.dt = dt; % TODO: not all systems have that - make it consistent

            sys.constraints.U = [0; 0.0001]; % Starting position and velocity
            sys.constraints.Y = [1; 0]; % Desired position and velocity
        case 'dc_motor'
            J = 0.01; % Inertia
            b = 0.1; % Damping coefficient
            K = 0.01; % Motor constant
            R = 1; % Resistance
            L = 0.5; % Inductance
            dt = 0.1; % Sampling time
            
            Ac = [-b/J K/J; -K/L -R/L];
            Bc = [0; 1/L];
            Cc = [1 0];
            Dc = 0;
            
            % Discretize the system
            % sys_cont = ss(Ac, Bc, Cc, Dc);
            % sys_disc = c2d(sys_cont, dt);
            % [Ad, Bd, Cd, Dd] = ssdata(sys_disc);
            
            sys.A = Ac;
            sys.B = Bc;
            sys.C = Cc;
            sys.D = Dc;
            
            sys.dims.state = size(sys.A, 1); % n - dim(x)
            sys.dims.input = size(sys.B, 2); % m - dim(u)
            sys.dims.output = size(sys.C, 1); % p - dim(y)

            sys.parameters.inertia = J;
            sys.parameters.damping = b;
            sys.parameters.motor_constant = K;
            sys.parameters.resistance = R;
            sys.parameters.inductance = L;
            sys.parameters.dt = dt;
            
            sys.constraints.U = [-10, 10]; % Voltage limits
            sys.constraints.Y = [-inf, inf]; % No speed limits
            
            sys.x_ini = [0; 0]; % Starting angular position and velocity
            sys.target = [10; 0]; % Desired angular position and velocity

        otherwise
            error('System type "%s" not recognized.', system_description);
    end
end
