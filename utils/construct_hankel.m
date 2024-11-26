function [H, H_flat] = custom_hankel(data, order)
    % CUSTOM_HANKEL - Constructs a Hankel matrix with shape 
    %                 [order x (T - order + 1)], where each entry is an m x 1 vector.
    %
    % Inputs:
    %   data    - A m x T matrix, where T is #time steps, and m is the input dimension.
    %   order   - The desired Hankel matrix order.
    %
    % Outputs:
    %   H       - A cell array of size [order x (T - order + 1)], where each entry is an m x 1 array.
    %   H_flat  - Column-wise vertically stacked Hankel matrix of size [(order * m) x (T - order + 1)].

    [m, T] = size(data); % m = input dimensions, T = time steps

    if order > T
        error("Data matrix has %d columns, but at least %d columns are required for order %d.", ...
            T, order, order);
    end

    num_columns = T - order + 1;

    % Preallocate H as a cell array
    H = cell(order, num_columns);

    % Populate the Hankel tensor
    for l = 1:order
        for k = 1:num_columns
            % Each entry is an m x 1 array
            H{l, k} = data(:, l + k - 1);
        end
    end

    % Create H_flat
    H_flat = zeros(order * m, num_columns); % Preallocate H_flat
    for k = 1:num_columns
        % Extract column blocks and vertically stack them
        column_block = cell2mat(H(:, k)); % Convert column block to matrix
        H_flat(:, k) = column_block(:);  % Flatten and store in H_flat
    end
end


