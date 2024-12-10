function deepc2_plot(time, y_sim, u_sim)
    % Plot outputs
    figure(1);
    p = size(y_sim, 1);
    
    for i=1:p
        subplot(p, 1, i);
        y_name = sprintf("y%d", i);
        plot(time, y_sim(1, :), 'r', 'LineWidth', 1.25, 'DisplayName', y_name); 
        xlabel('Iteration #');
        ylabel(sprintf('Output %d', i));
        grid on;
    end
    
    % Plot inputs
    figure(2);
    m = size(u_sim, 1);

    for i=1:m
        subplot(m, 1, i);
        u_name = sprintf("u%d", i);
        stairs(time, u_sim(1, :), 'r', 'LineWidth', 1.25, 'DisplayName', u_name);
        xlabel('Iteration #');
        ylabel(sprintf('Control input %d', i));
        grid on;
    end
end

