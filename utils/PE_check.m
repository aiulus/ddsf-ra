function [PE_input, is_PE] = generate_and_check_PE(L, m, desired_order, method)
    % Generates a persistently exciting (PE) input signal and checks if it's PE.
    %
    % Inputs:
    %   L             - Length of the data sequence
    %   m             - Number of inputs (columns) in the signal
    %   desired_order - Desired order of excitation for PE check
    %   method        - Method for generating the PE signal ('prbs', 'sine', 'white_noise', 'arma')
    %
    % Outputs:
    %   PE_input - Generated persistently exciting input signal
    %   is_PE    - Boolean indicating if the signal is PE of the specified order

    % Default signal generation method if not specified
    if nargin < 4
        method = 'prbs';
    end

    % Step 1: Generate the PE input signal based on the selected method
    switch lower(method)
        case 'prbs'  % Pseudo-Random Binary Sequence (PRBS)
            PE_input = idinput([L, m], 'prbs', [0, 1], [-1, 1]);
            disp('Generated PRBS signal as PE input');

        case 'sine'  % Sum of Sinusoids
            t = (0:L-1)';  % Time vector for signal length L
            freqs = linspace(0.1, 1, m);  % Choose a set of distinct frequencies
            PE_input = sum(sin(2 * pi * freqs .* t), 2);  % Sum of sinusoids
            disp('Generated Sum of Sinusoids as PE input');

        case 'white_noise'  % Gaussian White Noise
            PE_input = randn(L, m);  % Generate white noise with standard deviation 1
            disp('Generated Gaussian White Noise as PE input');

        case 'arma'  % Autoregressive Moving Average (ARMA) Signal
            % ARMA parameters
            ar_coeffs = [1, -0.5];  % AR part
            ma_coeffs = [1, 0.3];   % MA part
            PE_input = filter(ma_coeffs, ar_coeffs, randn(L, m));
            disp('Generated ARMA signal as PE input');

        otherwise
            error('Unknown method for generating PE input. Choose from ''prbs'', ''sine'', ''white_noise'', ''arma''.');
    end

    % Step 2: Construct Hankel matrix for PE check
    if size(PE_input, 2) ~= m
        PE_input = PE_input(:);
    end

    % Generate the Hankel matrix with the desired order
    H = hankel(PE_input(1:desired_order), PE_input(desired_order:end));

    % Step 3: Check the rank of the Hankel matrix for PE verification
    rank_H = rank(H);
    is_PE = rank_H >= desired_order;

    % Display result
    if is_PE
        disp(['The input is persistently exciting of order ', num2str(desired_order)]);
    else
        disp(['The input is NOT persistently exciting of order ', num2str(desired_order)]);
    end
end
