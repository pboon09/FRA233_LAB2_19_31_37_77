% Example data (replace this with your actual data)
time = cascade_try{2}.Values.Time;  % Time vector
response = cascade_try{2}.Values.Data;  % Response data

% Find the time index where the system transitions between the steady states
% You can manually identify this point or programmatically find the point
transition_index = find(time > 2.5, 1);  % Find the transition point after 2.5 seconds

% Split the response data into two segments

% For State 1 (before transition)
offset_state1 = response(1);  % First value of the response (initial offset)
response_state1 = response(1:transition_index);  % Offset by subtracting the initial value
time_state1 = time(1:transition_index);
final_value_state1 = response_state1(end);  % Final value (steady-state value)

% For State 2 (after transition)
initial_value_state2 = response(transition_index);  % Initial value for State 2
response_state2 = response(transition_index:end);  % Offset by subtracting the initial value of State 2
time_state2 = time(transition_index:end);
final_value_state2 = response_state2(end);  % Final value (steady-state value)

% Compute step response characteristics for the first state (before transition)
info_state1 = stepinfo(response_state1, time_state1);  % Compute stepinfo for State 1
peak_value_state1 = max(response_state1);  % Peak value of State 1
overshoot_state1 = ((peak_value_state1 - final_value_state1) / final_value_state1) * 100;  % Overshoot calculation

% Compute step response characteristics for the second state (after transition)
% info_state2 = stepinfo(response_state2, time_state2);  % Compute stepinfo for State 2
% peak_value_state2 = max(response_state2);  % Peak value of State 2
% overshoot_state2 = ((peak_value_state2 - initial_value_state2) / initial_value_state2) * 100;  % Overshoot calculation

% Display the results for both states
fprintf('State 1:\n');
fprintf('Final Value: %.4f\n', final_value_state1);
fprintf('Overshoot: %.4f%%\n', overshoot_state1);
fprintf('Rise Time: %.4f sec\n', info_state1.RiseTime);
fprintf('Peak Time: %.4f sec\n', info_state1.PeakTime);
fprintf('Settling Time: %.4f sec\n', info_state1.SettlingTime);

fprintf('\nState 2:\n');
fprintf('Final Value: %.4f\n', final_value_state2);
fprintf('Overshoot: %.4f%%\n', overshoot_state2);
fprintf('Rise Time: %.4f sec\n', info_state2.RiseTime);
fprintf('Peak Time: %.4f sec\n', info_state2.PeakTime);
fprintf('Settling Time: %.4f sec\n', info_state2.SettlingTime);

% Plot both states
figure;
subplot(2,1,1);
plot(time_state1, response_state1, 'b', 'LineWidth', 1.5);
title('State 1 - Before Transition');
xlabel('Time (s)');
ylabel('Response');

subplot(2,1,2);
plot(time_state2, response_state2, 'r', 'LineWidth', 1.5);
title('State 2 - After Transition');
xlabel('Time (s)');
ylabel('Response');
