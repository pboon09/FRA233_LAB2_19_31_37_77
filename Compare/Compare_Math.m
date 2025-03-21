time = kp_0_0645{1}.Values.Time;
response = kp_0_0645{1}.Values.Data;
time_math = kp_0_0645{2}.Values.Time;
response_math = kp_0_0645{2}.Values.Data;
final_value = 180*2*pi/360;

% Compute step response characteristics
info1 = stepinfo(response, time);
info2 = stepinfo(response_math, time_math);

upper_bound = final_value * 1.02;
lower_bound = final_value * 0.98;
settling_response1 = interp1(time, response, info1.SettlingTime);
settling_response2 = interp1(time, response, info2.SettlingTime);

% Compute steady-state response
steady_state_error1 = abs(final_value - response(end));
steady_state_error2 = abs(final_value - response_math(end));

% Display results
fprintf('System Response');
fprintf('Rise Time: %.4f sec\n', info1.RiseTime);
fprintf('Peak Time: %.4f sec\n', info1.PeakTime);
fprintf('Settling Time: %.4f sec\n', info1.SettlingTime);
fprintf('Percent Overshoot: %.4f%%\n', info1.Overshoot);
fprintf('Steady-State Error: %.4e\n', steady_state_error1);

fprintf('Math Model Response');
fprintf('Rise Time: %.4f sec\n', info2.RiseTime);
fprintf('Peak Time: %.4f sec\n', info2.PeakTime);
fprintf('Settling Time: %.4f sec\n', info2.SettlingTime);
fprintf('Percent Overshoot: %.4f%%\n', info2.Overshoot);
fprintf('Steady-State Error: %.4e\n', steady_state_error2);


% Plot response
figure;

% --- First Subplot: System Response (Time vs Response) ---
subplot(1,2,1);
plot(time, response, 'b', 'LineWidth', 1.5); hold on;
plot(info1.PeakTime, max(response), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Peak point
plot(info1.SettlingTime, settling_response1, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

yline(final_value, '--k');
yline(upper_bound, '--g');
yline(lower_bound, '--g');

xlabel('Time (s)');
ylabel('Response');
title('System Response');
grid on;
legend({'Math Model Response', 'Peak Time', 'Settling Time', 'Final Value'}, 'FontSize', 9);
legend('Position', [0.3, 0.215, 0.2, 0.2]);

str = {
    ['Rise Time: ', num2str(info.RiseTime, '%.4f'), ' sec'],
    ['Peak Time: ', num2str(info.PeakTime, '%.4f'), ' sec'],
    ['Settling Time: ', num2str(info.SettlingTime, '%.4f'), ' sec'],
    ['Percent Overshoot: ', num2str(info.Overshoot, '%.4f'), '%'],
    ['Steady-State Error: ', num2str(steady_state_error, '%.4e')]
};

% Display the text box avoiding legend overlap
text(12.28, 0.03, str, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);

% --- Second Subplot: Mathematical Model Response (Time Math vs Response Math) ---
subplot(1,2,2);
plot(time_math, response_math, 'r', 'LineWidth', 1.5); hold on;
plot(info2.PeakTime, max(response_math), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Peak point
plot(info2.SettlingTime, settling_response2, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

yline(final_value, '--k');
yline(upper_bound, '--g');
yline(lower_bound, '--g');

xlabel('Time (s)');
ylabel('Response');
title('Mathematical Model Response');
grid on;
legend({'Math Model Response', 'Peak Time', 'Settling Time', 'Final Value'}, 'FontSize', 9);
legend('Position', [0.739, 0.215, 0.2, 0.2]);

str = {
    ['Rise Time: ', num2str(info.RiseTime, '%.4f'), ' sec'],
    ['Peak Time: ', num2str(info.PeakTime, '%.4f'), ' sec'],
    ['Settling Time: ', num2str(info.SettlingTime, '%.4f'), ' sec'],
    ['Percent Overshoot: ', num2str(info.Overshoot, '%.4f'), '%'],
    ['Steady-State Error: ', num2str(steady_state_error, '%.4e')]
};

% Display the text box avoiding legend overlap
text(12.28, 0.03, str, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);
