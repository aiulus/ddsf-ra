function [time, u_hist, y_hist] = mpcMainF(systype)
    sys = deepc2_systems(systype);
    sys.target = sys.params.target;

    % Extract system matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    % MPC Parameters
    dt = sys.params.dt; % Sampling time from system parameters
    t_p = sys.run_config.T_f; % Prediction horizon (steps)
    t_c = 5; % Control horizon (steps)
    
    % Create the plant as a state-space object
    plant = ss(A, B, C, D);
    
    % Initialize the MPC controller
    mpc_controller = mpc(plant, dt, t_p, t_c);
    
    % Set constraints
    mpc_controller.MV.Min = sys.constraints.U(1); % Minimum input
    mpc_controller.MV.Max = sys.constraints.U(2); % Maximum input
    
    if isfield(sys.constraints, 'Y') && ~isempty(sys.constraints.Y)
        mpc_controller.OV.Min = sys.constraints.Y(1); % Minimum output (if defined)
        mpc_controller.OV.Max = sys.constraints.Y(2); % Maximum output (if defined)
    end

    % Tuning parameters
    mpc_controller.Weights.ManipulatedVariables = 1e-4; % Penalize input magnitude
    mpc_controller.Weights.ManipulatedVariablesRate = 1e-3; % Penalize rate of input change
    mpc_controller.Weights.OutputVariables = 1; % Penalize output deviation
    
    % Simulation parameters
    t_final = 5; % Total simulation time [s]
    N_simulation = t_final / dt; % Number of time steps
    x_ini = sys.params.x_ini; % Initial state from system definition
    x_history = zeros(size(A, 1), N_simulation + 1); % Storage for states
    x_history(:, 1) = x_ini;
    % y_ini = sys.C * x_ini;
    y_history = zeros(size(C,1), N_simulation + 1);
    % y_history(:, 1) = y_ini;
    u_history = zeros(size(B, 2), N_simulation + 1); % Storage for control inputs
    r = sys.params.target; % Target reference

    % Simulation loop
    for k = 1:N_simulation
        % Compute optimal control action
        %u = mpcmove(mpc_controller, mpcstate(mpc_controller), x, r);
        x = x_history(:, k);
        y = C * x + D * u_history(:, k);
        u = mpcmove(mpc_controller, mpcstate(mpc_controller), y, r);
    
        % Update system state
        x = A * x + B * u;
    
        % Log state and input
        x_history(:, k + 1) = x;
        y_history(:, k) = y;
        u_history(:, k) = u;
    end

    % Time vector
    time = (1:N_simulation);
    u_hist = u_history(:, 2:end);
    y_hist = y_history(:, 2:end);
    
    % deepc2_plot(t, y_history(:, 2:end), u_history(:, 2:end), sys)
end

