function deepc2_plot(time, y_sim, u_sim, ypre1, ypre2, ypre3, ypre4)
    % Plot outputs
    figure(1);
    tiledlayout(4, 1);
    
    % Plot ysim and predictions for y1
    nexttile; hold on;    
    scatter(time, y_sim(1, :), 'r', 'Marker', '^', 'DisplayName', 'Real y1'); 
    plot(time, ypre1(1, :), 'b', 'DisplayName', 'Predicted1');
    plot(time, ypre2(1, :), 'g', 'DisplayName', 'Predicted2');
    plot(time, ypre3(1, :), 'k', 'DisplayName', 'Predicted3');
    plot(time, ypre4(1, :), 'm', 'DisplayName', 'Predicted4');
    title('Output y1'); xlabel('Time'); ylabel('y1');
    legend('Location', 'best'); grid on; 
    
    % Plot ysim and predictions for y2
    nexttile; hold on;
    scatter(time, y_sim(2, :), 'r', 'Marker', '^', 'DisplayName', 'Real y2'); 
    plot(time, ypre1(2, :), 'b', 'DisplayName', 'Predicted1');
    plot(time, ypre2(2, :), 'g', 'DisplayName', 'Predicted2');
    plot(time, ypre3(2, :), 'k', 'DisplayName', 'Predicted3');
    plot(time, ypre4(2, :), 'm', 'DisplayName', 'Predicted4');
    title('Output y2'); xlabel('Time'); ylabel('y2');
    legend('Location', 'best'); grid on;
    
    % Plot ysim and predictions for y3
    nexttile; hold on;
    scatter(time, y_sim(3, :), 'r', 'Marker', '^', 'DisplayName', 'Real y3'); 
    plot(time, ypre1(3, :), 'b', 'DisplayName', 'Predicted1');
    plot(time, ypre2(3, :), 'g', 'DisplayName', 'Predicted2');
    plot(time, ypre3(3, :), 'k', 'DisplayName', 'Predicted3');
    plot(time, ypre4(3, :), 'm', 'DisplayName', 'Predicted4');
    title('Output y3'); xlabel('Time'); ylabel('y3');
    legend('Location', 'best'); grid on;
    
    % Plot inputs
    figure(2);

    subplot(3, 1, 1); % 3 rows, 1 column, 1st subplot
    stairs(time, u_sim(1, :), 'r', 'LineWidth', 1.5); % 'stairs' function creates a step plot
    ylabel('u1');
    
    subplot(3, 1, 2); % 3 rows, 1 column, 2nd subplot
    stairs(time, u_sim(2, :), 'r', 'LineWidth', 1.5);
    ylabel('u2');
end

