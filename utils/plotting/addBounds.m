function addBounds(time, bounds)
    if bounds(1) ~= -inf, plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound'); end
    if bounds(2) ~= inf, plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound'); end
end