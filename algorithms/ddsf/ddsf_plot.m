function ddsf_plot(time, logs, lookup)    
    sys = lookup.sys;
    ul_hist = logs.u_d;
    u_hist = logs.u;
    y_hist = logs.y;
        
    figure(1);
    m = size(u_hist, 1);
    tiledlayout(m, 1); % Use tiled layout for better control
    
    % Plot learning vs safe inputs
    for i = 1:m
        nexttile;
        stairs(time, ul_hist(i, :), 'r', 'LineWidth', 1.25, 'DisplayName', sprintf('ul[%d]', i));
        hold on;
        stairs(time, u_hist(i, :), 'b', 'LineWidth', 1.5, 'DisplayName', sprintf('u[%d]', i));
    
        bounds = sys.constraints.U(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
    
        title(sprintf('Learning vs Safe Input %d', i));
        xlabel('t');
        ylabel(sprintf('Input %d', i));
        grid on;
        legend show;
        hold off;
    end
        
    figure(2);
    p = size(y_hist, 1);
    tiledlayout(m + p, 1); % Combine all inputs and outputs into a single layout
    
    % Plot learning vs safe inputs
    for i = 1:m
        nexttile;
        stairs(time, ul_hist(i, :), 'r', 'LineStyle', ':','LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
        hold on;
        stairs(time, u_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('u[%d]', i));
    
        bounds = sys.constraints.U(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
    
        title(sprintf('Learning vs Safe Input %d', i));
        xlabel('t');
        ylabel(sprintf('Input %d', i));
        grid on;
        legend show;
        hold off;
    end
    
    % Plot system outputs
    for i = 1:p
        nexttile; hold on;
        plot(time, y_hist(i, :), 'r', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
        bounds = sys.constraints.Y(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'r--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'r--', 'DisplayName', 'Upper Bound');
        end
        if lookup.opt_params.target_penalty
            pi = sys.params.target(i);
            plot(time, pi * ones(size(time)), 'g--', 'DisplayName', 'Target');
        end
    
        title(sprintf('System Output %d', i));
        xlabel('t');
        ylabel(sprintf('Output %d', i));
        grid on;
        legend show;
        hold off;
    end
    
    sgtitle('Comparison of Learning Inputs, Safe Inputs, and Outputs');

end

function single_plots(time, logs, sys)
    ul_hist = logs.u_d;
    u_hist = logs.u;
    y_hist = logs.y;

    figure(1);
    p = size(y_hist, 1);
    for i = 1:p
        subplot(p, 1, i);
        plot(time, y_hist(i, :), 'r', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
        bounds = sys.constraints.Y(i, :);
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
        title(sprintf('System Output %d', i));
        xlabel('t');
        ylabel(sprintf('Output %d', i));
        grid on;
        legend show;
    end
    sgtitle('System Outputs');
    
        
    figure(2);
    m = size(u_hist, 1);
    tiledlayout(m, 1); % Use tiled layout for better control
    
    for i = 1:m
        nexttile; % Equivalent to subplot
        stairs(time, ul_hist(i, :), 'r', 'LineWidth', 1.25, 'DisplayName', sprintf('ul[%d]', i));
        bounds = sys.constraints.U(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            hold on;
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            hold on;
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
    
        title(sprintf('Learning Input %d', i));
        xlabel('t');
        ylabel(sprintf('Learning Input %d', i));
        grid on;
        legend show;
        hold off;
    end
    
    sgtitle('Learning Inputs');
    
    
    figure(3);
    m = size(u_hist, 1);
    tiledlayout(m, 1); % Use tiled layout for better control
    
    for i = 1:m
        nexttile;
        stairs(time, u_hist(i, :), 'b', 'LineWidth', 1.5, 'DisplayName', sprintf('u[%d]', i));
        bounds = sys.constraints.U(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            hold on;
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            hold on;
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
    
        title(sprintf('Safe Input %d', i));
        xlabel('t');
        ylabel(sprintf('Safe Input %d', i));
        grid on;
        legend show;
        hold off;
    end
    
    sgtitle('Safe Inputs');
    
end