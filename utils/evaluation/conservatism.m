function c_d = conservatism(y_d, yl_d, lb, ub)
    % CONSERVATISM computes the conservatism of a safety filter 
    % using percentile-based range and range-normalized variance.
    %
    % Inputs:
    %   y_d   - Original signal for one dimension (1 x T_sim).
    %   yl_d  - Filtered (safe) output signal (same size as y_d).
    %
    % Output:
    %   c_d   - Scalar conservatism metric (0 to 1, where 1 is highly conservative).

    cut_percentage = 10;

    % Ensure signals are compatible
    if size(yl_d) ~= size(y_d)
        error('Original and filtered signals must have the same dimensions.');
    end

    full_range = ub - lb;

    p = cut_percentage / 2;
    covered_range = prctile(yl_d, 100 - p) - prctile(yl_d, p);

    covered_range_percentage = covered_range / full_range;

    variance_yl = normalized_variance(yl_d); 

    % Conservatism metric
    c_d = covered_range_percentage / (variance_yl + 1);
end
