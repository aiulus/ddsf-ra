% Initialize parameters
system_type = "cruise_control";
T_sim = 5;
initial_params = struct('Q', 0.1, 'R', 0.1, 'T_ini', 5, 'N', 10);
constraints = struct('Q', 1e8, 'R', 1e8, 'T_ini', @(N) N - 1, 'N', 80);
best_loss = Inf;
best_params = initial_params;
results = {};

% Function to evaluate the system
evaluate_system = @(params) runParamDPC(system_type, params.Q, params.R, params.T_ini, params.N, T_sim);

% Function to perform binary search
function [opt_value, opt_loss] = binary_search(param_name, low, high, fixed_params, evaluate_system, best_loss)
    tol = 1e-3; % Tolerance for convergence
    while (high - low) > tol
        mid = (low + high) / 2;
        params = fixed_params;
        params.(param_name) = mid;
        loss = evaluate_system(params).loss;
        if loss < best_loss
            high = mid;
        else
            low = mid;
        end
    end
    opt_value = (low + high) / 2;
    final_params = fixed_params;
    final_params.(param_name) = opt_value;
    opt_loss = evaluate_system(final_params).loss;
end

% Iterate over each parameter
param_names = fieldnames(initial_params);
for i = 1:length(param_names)
    param_name = param_names{i};
    current_value = initial_params.(param_name);
    fixed_params = best_params;
    
    % Exponential increase
    while true
        new_value = current_value * 10;
        if isnumeric(constraints.(param_name)) && new_value > constraints.(param_name)
            break;
        end
        fixed_params.(param_name) = new_value;
        loss = evaluate_system(fixed_params).loss;
        results = [results; {fixed_params, loss}];
        if loss < best_loss
            best_loss = loss;
            best_params = fixed_params;
            current_value = new_value;
        else
            break;
        end
    end
    
    % Binary search
    if current_value > initial_params.(param_name)
        low = current_value / 10;
        high = current_value;
        [opt_value, opt_loss] = binary_search(param_name, low, high, fixed_params, evaluate_system, best_loss);
        fixed_params.(param_name) = opt_value;
        results = [results; {fixed_params, opt_loss}];
        if opt_loss < best_loss
            best_loss = opt_loss;
            best_params = fixed_params;
        end
    end
end

% Sort results by loss
results = sortrows(results, 2);

% Display results
for j = 1:size(results, 1)
    params = results{j, 1};
    loss = results{j, 2};
    fprintf('Q=%.2e, R=%.2e, T_ini=%d, N=%d, Loss=%.4f\n', params.Q, params.R, params.T_ini, params.N, loss);
end

% Display best parameters
fprintf('Best Parameters: Q=%.2e, R=%.2e, T_ini=%d, N=%d, Loss=%.4f\n', best_params.Q, best_params.R, best_params.T_ini, best_params.N, best_loss);
