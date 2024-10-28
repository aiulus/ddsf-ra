function sys = quadrotor()
    sys.name = "6dof-quadrotor";

    %% Parameters
    mass = 0.2; % [kg]
    g = 9.81; % [m/sec^2]
    T_s = 0.1; % [sec]
    u_min = [-1; -0.1; -0.1; -0.1]; % N(.m)^4
    u_max = [1; 0.1; 0.1; 0.1]; % N(.m)^4
    theta_min = -0.2; % [rad]
    theta_max = 0.2; % [rad]
    phi_min = -0.2; % [rad]
    phi_max = 0.2; % [rad]
    psi_min = -0.2; % [rad]
    psi_max = 0.2; % [rad]
    I = repmat(10.^(-3), 3, 1);
    X_min = repmat(-1, 3, 1);
    X_max = ones(3, 1);

    %% State-space Matrices
    sys.nx = 12;
    sys.nu = 4;
    sys.ny = 6;

    A_i = [1, 2, 3, 10, 11, 12];
    A_j = [4, 5, 6, 7, 8, 9];
    A_val = ones(6, 1);
    A = sparse(A_i, A_j, A_val, 12, 12);
    sys.model.A = A;

    B_i = [9, 4, 5, 6];
    B_j = [1, 2, 3, 4];
    B_val = [mass.^(-1), I(1).^(-1), I(2).^(-1), I(3).^(-1)];
    B = sparse(B_i, B_j, B_val, 12, 4);
    sys.model.B = B;
    

    sys.model.C = [1, 0; 0, 1];
    sys.model.D = 0;
    sys.model.f = @(x, u) sys.model.A*x + sys.model.B*u;  % dynamics
    sys.model.h = @(x, u) sys.model.C*x + sys.model.D*u;  % measurement