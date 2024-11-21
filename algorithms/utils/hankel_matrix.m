%% TODO: Remove / Obsolete (replaced by utils/PEness_check.m)

% Implements sanity checks for the construction of Hankel matrices
function H = hankel_matrix(data,L)
    %   Constructs a Hankel matrix (H_L(u)/H_L(y)) from input/output data
    %
    %   Inputs:
    %   data - Vector of input/output data (column vector)
    %   L - Order of the Hankel matrix
    
    % Validate input
    if ~isvector(data)
        error('Input data must be a vector');
    end
    
    if L > length(data)
        error('Order L must be less than or equal to the length of the data vector.');
    end
    
    H = hankel(data(1:L), data(L:end));
end

