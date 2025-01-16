function deepc3plot(time, y_sim, u_sim, sys)
    % Plot outputs
    figure(1);
    p = size(y_sim, 1);
    sys.target = sys.params.target;

    for i=1:p
        subplot(p, 1, i);
        y_name = sprintf("y%d", i);
        hold on;
        plot(time, y_sim(1, :), 'r', 'LineWidth', 1.25, 'DisplayName', y_name); 
        xlabel('Iteration #');
        ylabel(sprintf('Output %d', i));

        target = sys.target * ones(1, size(time, 2));
        plot(time, target, 'm--', 'DisplayName', 'Target');
       
        grid on; legend show; hold off;
    end
    sgtitle("Control inputs over time");
    
    % Plot inputs
    figure(2);
    m = size(u_sim, 1);

    for i=1:m
        subplot(m, 1, i);
        u_name = sprintf("u%d", i);
        hold on;
        stairs(time, u_sim(1, :), 'r', 'LineWidth', 1.25, 'DisplayName', u_name);
        xlabel('Iteration #');
        ylabel(sprintf('Control input %d', i));

        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
        grid on; legend show; hold off;
    end
    sgtitle("System outputs over time");
    figure(1);
    matlab2tikz('deepc-cruise_control-outputs.tex');
    figure(2);
    matlab2tikz('deepc-cruise_control-inputs.tex');
end

