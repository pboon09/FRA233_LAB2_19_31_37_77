i = 2;
time = P_025_180.Time;
response = P_025_180.Data;
final_value = (i*90)*2*pi/360;

% Compute step response characteristics
info = stepinfo(response, time);

upper_bound = final_value * 1.02;
lower_bound = final_value * 0.98;
settling_response = interp1(time, response, info.SettlingTime);

% Compute steady-state response
steady_state_response = response(end);
steady_state_error = abs(final_value - steady_state_response);

% Display results
fprintf('Rise Time: %.4f sec\n', info.RiseTime);
fprintf('Peak Time: %.4f sec\n', info.PeakTime);
fprintf('Settling Time: %.4f sec\n', info.SettlingTime);
fprintf('Percent Overshoot: %.4f%%\n', info.Overshoot);
fprintf('Steady-State Error: %.4e\n', steady_state_error);

% Plot response
figure;
plot(time, response, 'b', 'LineWidth', 1.5); hold on;
plot(info.PeakTime, max(response), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Peak point
plot(info.SettlingTime, settling_response, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');

yline(final_value, '--k');
yline(upper_bound, '--g');
yline(lower_bound, '--g');

xlabel('Time (s)', 'FontSize', 10);
ylabel('Response', 'FontSize', 10);
title('System Response', 'FontSize', 14);
grid on;
legend({'System Response', 'Peak Time', 'Settling Time', 'Final Value'}, 'FontSize', 14);
legend('Position', [0.73, 0.28, 0.2, 0.2]);

str = {
    ['Rise Time: ', num2str(info.RiseTime, '%.4f'), ' sec'],
    ['Peak Time: ', num2str(info.PeakTime, '%.4f'), ' sec'],
    ['Settling Time: ', num2str(info.SettlingTime, '%.4f'), ' sec'],
    ['Percent Overshoot: ', num2str(info.Overshoot, '%.4f'), '%'],
    ['Steady-State Error: ', num2str(steady_state_error, '%.4e')]
};

% Display the text box avoiding legend overlap
text(15.3, 0.2, str, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 14);