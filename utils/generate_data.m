function [u_d, y_d] = generate_data(A, B, C, D, L)
    %   Inputs:
    %   A, B, C, D - System matrices (default: C=1, D=0 for SISO systems)
    %   L - data length
    %   m - scaling factor
    %
    %   Outputs:
    %   u_d - input data
    %   y_d - output data

    % Set default values for SISO systems
    if nargin < 4 || isempty(C), C = 1; end
    if nargin < 5 || isempty(D), D = 0; end

    % Generate a persistently exciting, pseudo-random control input
    % m = L^2;
    % PE_input = m * (idinput([size(B, 2), L], 'prbs') + 1);
    PE_input = idinput([L, size(B,2)], 'prbs', [0, 1], [-1,1]).';
    disp("PE_input: "); disp(PE_input);

    % Initialize input-output storage
    u_d = zeros(size(B, 2), L);
    y_d = zeros(size(C, 1), L);
    x_data = zeros(size(A, 1), 1); % Initial state

    % Generate data by simulating the system on random inputs for L steps
    for i = 1:L
        u_d(:, i) = PE_input(:, i);
        y_d(:, i) = C * x_data + D * u_d(:, i);
        x_data = A * x_data + B * u_d(:, i);
    end
end

