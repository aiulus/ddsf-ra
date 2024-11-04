%% CRUISE CONTROL: CONSTANT SPEED
function [u_d, y_d] = datagen_cruise_control(L)
    % Input
    % L - # Data points to generate

    % Parameters
    m = 1000; % Vehicle mass [kg]
    b = 50; % Damping coefficient [N*s/m]
    dt = 0.1; % Time step for discretization
    
    % State-Space Matrices
    A = 1 - (b * dt) / m;
    B = dt / m;
    C = 1;
    D = 0;
    
    [u_d, y_d] = generate_data(A, B, C, D, L);
end