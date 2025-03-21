% Example data (replace this with your actual data)
time = cascade_try{2}.Values.Time;  % Time vector
response = cascade_try{2}.Values.Data;  % Response data
ref = cascade_try{6}.Values.Data;
abs_error = abs(ref - response);
time_state = [2,2,2,4];

prev = 1;
prev_time = 0;
for i = 1:length(time_state)
    split_index = (time_state(i)*1000) + prev_time; 
    fprintf('Split Index %d:\n', split_index);
    % Split the response data and time vector
    response_state = response(prev:split_index); 
    time_state_segment = time(prev:split_index);
    idx_range = find((time1 >= t_start) & (time1 < t_end));

    if response(prev) > response(split_index)
        response_state = -response_state;
    end
    % Calculate the final value (steady-state value) for the current state
    final_value_state = response_state(end);
   
    % Compute the step response characteristics for the current state
    info_state = stepinfo(response_state, time_state_segment);  % Compute stepinfo
    peak_value_state = max(response_state);  % Peak value of the current state
    overshoot_state = ((peak_value_state - final_value_state) / final_value_state) * 100;  % Overshoot calculation
    
    

    prev = split_index;  % The next state starts after the current split index
    prev_time = split_index;  % Update previous time for the next split
    
    % Display the results for the current state
    fprintf('State %d:\n', i);
    fprintf('Final Value: %.4f\n', final_value_state);
    fprintf('Overshoot: %.4f%%\n', overshoot_state);
    fprintf('Rise Time: %.4f sec\n', info_state.RiseTime);
    fprintf('Peak Time: %.4f sec\n', info_state.PeakTime);
    fprintf('Peak Value: %.4f sec\n', peak_value_state);
    fprintf('Settling Time: %.4f sec\n', info_state.SettlingTime);
    % fprintf('Error: %.4f\n', abs_error);
    fprintf('------------------\n');
    % Update the previous values for the next iteration
   
end
% Split the response data into two segments

% Plot both states
% figure;
% subplot(2,1,1);
% plot(time_state1, response_state1, 'b', 'LineWidth', 1.5);
% title('State 1 - Before Transition');
% xlabel('Time (s)');
% ylabel('Response');
