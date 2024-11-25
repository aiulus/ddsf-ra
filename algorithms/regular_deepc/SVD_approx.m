function H_approx = SVD_approx(H)
    [U, S, V] = svd(H, 'econ');
    
    % Determine rank cutoff based on energy retention (95% as an example)
    energy_threshold = 0.95;

    singular_values = diag(S);
    energy_retained = cumsum(singular_values.^2) / sum(singular_values.^2);
    rank_cutoff = find(energy_retained >= energy_threshold, 1);
        
    % Reconstruct low-rank approximations
    H_approx = U(:, 1:rank_cutoff) * S(1:rank_cutoff, 1:rank_cutoff) * V(:, 1:rank_cutoff)';
end

