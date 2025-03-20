time1 = t_multiply1_5{1}.Values.Time;
response1 = t_multiply1_5{1}.Values.Data;
time2 = t_multiply2{1}.Values.Time;
response2 = t_multiply2{1}.Values.Data;
time3 = t{1}.Values.Time;
response3 = t{1}.Values.Data;
time4 = t_divide1_5{1}.Values.Time;
response4 = t_divide1_5{1}.Values.Data;
time5 = t_divide2{1}.Values.Time;
response5 = t_divide2{1}.Values.Data;

final_value = 180*2*pi/360;

% Compute step response characteristics
info1 = stepinfo(response1, time1);
info2 = stepinfo(response2, time2);
info3 = stepinfo(response3, time3);
info4 = stepinfo(response4, time4);

upper_bound = final_value * 1.02;
lower_bound = final_value * 0.98;

% Plot response
figure;

plot(time1, response1, 'g', 'LineWidth', 1.5); hold on;
plot(time2, response2, 'r', 'LineWidth', 1.5);
plot(time3, response3, 'b', 'LineWidth', 1.5);
plot(time4, response4, 'm', 'LineWidth', 1.5);
plot(time5, response5, 'c', 'LineWidth', 1.5);

yline(final_value, '--k');
yline(upper_bound, '--g');
yline(lower_bound, '--g');

xlabel('Time (s)');
ylabel('Response');
title('System Response with Different Time Constant');
grid on;
legend({'T*2', 'T*1.5', 'T', 'T/1.5', 'T/2'}, 'FontSize', 10);
legend('Position', [0.3, 0.215, 0.2, 0.2]);