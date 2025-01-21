function plotDDSF(time, logs, lookup)    
    sys = lookup.sys;
    ul_hist = logs.ul_t; % s
    u_hist = logs.u;
    y_hist = logs.y;
    yl_hist = logs.yl;
    loss_hist = logs.loss;
        
    figure(1);
    m = size(u_hist, 1);
    tiledlayout(m, 1); % Use tiled layout for better control
  
    
    % Plot learning vs safe inputs
    for i = 1:m
        nexttile;
        stairs(0:size(ul_hist, 2) - 1, ul_hist(i, :), 'r', 'LineStyle', ':','LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
        hold on;
        stairs(0:size(u_hist, 2) - 1, u_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('u[%d]', i));
    
        bounds = sys.constraints.U(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'm--', 'DisplayName', 'Upper Bound');
        end
    
        title(sprintf('Learning vs Safe Input %d', i));
        xlabel('t');
        ylabel(sprintf('Input %d', i));
        grid on;
        legend show;
        hold off;
    end        
    sgtitle('Learning Inputs vs. Safe Inputs');

    figure(2);
    p = size(y_hist, 1);
    tiledlayout(p, 1); % Combine all inputs into a single layout

    for i = 1:p
        nexttile; hold on;
        plot(time, y_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
        stairs(0:size(yl_hist, 2) - 1, yl_hist(i, :), 'r', 'LineStyle', ':','LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
        hold on;

        bounds = sys.constraints.Y(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'm--', 'DisplayName', 'Upper Bound');
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
    sgtitle("Resulting Outputs");


    figure(3);
    hold on;
    plot(0:size(loss_hist, 2) - 1, loss_hist(1, :), 'r', 'LineWidth', 1.25, 'DisplayName', 'delta_u');
    plot(0:size(loss_hist, 2) - 1, loss_hist(2, :), 'b', 'LineWidth', 1.25, 'DisplayName', 'delta_u + distance to target convergence point');
    grid on; legend show; hold off;
    sgtitle('Losses');

    figure(1);
    matlab2tikz('quad_inputs_t50.tex');
    figure(2);
    matlab2tikz('quad_outputs_t50.tex');
end
