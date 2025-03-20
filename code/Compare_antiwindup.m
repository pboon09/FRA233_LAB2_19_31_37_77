time1 = P_025_180.Time;
response1 = P_025_180.Data;
time2 = no_antiwindup_p_025{2}.Values.Time;
response2 = no_antiwindup_p_025{2}.Values.Data;

final_value = 180*2*pi/360;

% Compute step response characteristics
info1 = stepinfo(response1, time1);
info2 = stepinfo(response2, time2);

upper_bound = final_value * 1.02;
lower_bound = final_value * 0.98;

% Plot response
figure;

plot(time1, response1, 'g', 'LineWidth', 1.5); hold on;
plot(time2, response2, 'r', 'LineWidth', 1.5);

yline(final_value, '--k');
yline(upper_bound, '--g');
yline(lower_bound, '--g');

xlabel('Time (s)');
ylabel('Response');
title('System Response with Different Time Constant');
grid on;
legend({'T*2', 'T*1.5'}, 'FontSize', 10);
legend('Position', [0.3, 0.215, 0.2, 0.2]);