function addBounds(time, bounds, configname)
    factor = filename2param(configname, 'constr');
    if isempty(factor)
        factor = 1;
    end

    % DEBUG STATEMENT
    fprintf("\n------------- addBounds.m ------------- \n");
    fprintf("Received: \n");
    disp('Received lower bound: '); disp(bounds(1));
    disp('Received upper bound: '); disp(bounds(2));
    disp("Computed factor: "); disp(factor); 

    bounds = updateBounds(bounds, factor);

    disp('Updaed lower bound: '); disp(bounds(1));
    disp('Updated upper bound: '); disp(bounds(2));


    if bounds(1) ~= -inf && abs(bounds(1)) < 1e+8
        plot(time, bounds(1) * ones(size(time)), 'm--', 'LineWidth', 1.25, 'DisplayName', 'Lower Bound'); 
    end
    if bounds(2) ~= inf && abs(bounds(2)) < 1e+8
        plot(time, bounds(2) * ones(size(time)), 'm--', 'LineWidth', 1.25, 'DisplayName', 'Upper Bound'); 
    end
end