T_sim = 25;
system_type = "cruise_control";
Q_values = [0.1, 1, 10, 1e+4];
R_values = [0.01, 0.1, 1];
T_ini_values = [5, 10, 15];
N_values = [10, 20, 30];
valcount = size(Q_values, 2)*size(R_values, 2)*size(T_ini_values, 2)*size(N_values, 2);
counter = 1;

best_loss = Inf;
best_params = struct('Q', NaN, 'R', NaN, 'T_ini', NaN, 'N', NaN);

start_time = tic; 

for Q = Q_values
    for R = R_values
        for T_ini = T_ini_values
            for N = N_values
                try
                    fprintf("------------------- " + ...
                    "Grid Search - Attempting to evaluate %d / %d " + ...
                    "total parameter combinations -------------------\n", counter, valcount);
                    logs = runParamDPC(system_type, Q, R, T_ini, N, T_sim);
                    final_loss = logs.loss(T_sim);
                    if isnan(final_loss)
                        fprintf('Optimizer failed for Q=%.2f, R=%.2f, T_ini=%d, N=%d\n', Q, R, T_ini, N);
                        final_loss = Inf;
                    end
                    if final_loss < best_loss
                        best_loss = final_loss;
                        best_params = struct('Q', Q, 'R', R, 'T_ini', T_ini, 'N', N);
                    end
                catch ME
                    fprintf('Error for Q=%.2f, R=%.2f, T_ini=%d, N=%d: %s\n', Q, R, T_ini, N, ME.message);
                end
                
                elapsed_time = toc(start_time);

                % Estimate total time and remaining time
                avg_time_per_iteration = elapsed_time / counter;
                estimated_total_time = avg_time_per_iteration * valcount;
                remaining_time = estimated_total_time - elapsed_time;
                
                % Display time estimations
                fprintf('Elapsed Time: %.2f seconds\n', elapsed_time);
                fprintf('Estimated Total Time: %.2f seconds\n', estimated_total_time);
                fprintf('Estimated Remaining Time: %.2f seconds\n', remaining_time);

                counter = counter + 1;
            end
        end
    end
end
