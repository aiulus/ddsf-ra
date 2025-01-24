function V_scaled = updateBounds(V, factor)
    fprintf("\n------------- updateBounds.m -------------\n");
    fprintf("Received bounds: "); disp(V);
    V_min = V(:, 1); V_max = V(:, 2);
    factor = factor / 2;
    V_scaled_min = V_min - factor .* abs(V_min);
    V_scaled_max = V_max + factor .* abs(V_max);
    V_scaled = [V_scaled_min, V_scaled_max];
        
    fprintf("\n------------- updateBounds.m -------------\n");
    fprintf("Updated bounds: "); disp(V_scaled);
end