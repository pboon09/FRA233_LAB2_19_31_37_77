% time5 = P_075_180.Time;
% response5 = P_075_180.Data;
% time4 = P_025_180.Time;
% response4 = P_025_180.Data;
% time3 = default{2}{2}.Values.Time;
% response3 = default{2}{2}.Values.Data;
% time2 = P_125_180.Time;
% response2 = P_125_180.Data;
% time1 = P_175_180.Time;
% response1 = P_175_180.Data;

% time5 = i_025_180.Time;
% response5 = i_025_180.Data;
% time4 = i_075_180.Time;
% response4 = i_075_180.Data;
% time3 = default{2}{2}.Values.Time;
% response3 = default{2}{2}.Values.Data;
% time2 = i_125_180.Time;
% response2 = i_125_180.Data;
% time1 = i_175_180.Time;
% response1 = i_175_180.Data;

time5 = d_0_25{2}{2}.Values.Time;
response5 = d_0_25{2}{2}.Values.Data;
time4 = d_0_75{2}{2}.Values.Time;
response4 = d_0_75{2}{2}.Values.Data;
time3 = default{2}{2}.Values.Time;
response3 = default{2}{2}.Values.Data;
time2 =  d_1_25{2}{2}.Values.Time;
response2 =  d_1_25{2}{2}.Values.Data;
time1 =  d_1_75{2}{2}.Values.Time;
response1 = d_1_75{2}{2}.Values.Data;

final_value = 180*2*pi/360;

% Compute step response characteristics
info1 = stepinfo(response1, time1);
info2 = stepinfo(response2, time2);
info3 = stepinfo(response3, time3);
info4 = stepinfo(response4, time4);
info5 = stepinfo(response5, time5);

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
title('System Response with Different Kd');
grid on;
legend({'D + 75%', 'D + 25%', 'D', 'D - 25%', 'D - 75%'}, 'FontSize', 10);
legend('Position', [0.3, 0.215, 0.2, 0.2]);