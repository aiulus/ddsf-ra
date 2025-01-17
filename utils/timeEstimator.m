function timeEstimator(elapsed, t, T)
    % t -   Current iteration
    % T -   Total #iterations
    avg_per_run = elapsed / t - 1;
    rem = avg_per_run * (T - t +1);
    mins = floor(rem / 60);
    secs = mod(rem, 60);
    fprintf('------------------- Estimated time remaining: %d m %.0f s remaining\n', mins, secs);
end