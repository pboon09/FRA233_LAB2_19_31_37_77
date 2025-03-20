% Extract time and response data
time1 = test2{2}.Values.Time;  % Time values
response1 = test2{2}.Values.Data;  % Measured response
ref1 = test2{6}.Values.Data;  % Reference signal

% Ensure data is column vectors
time1 = time1(:);
response1 = response1(:);
ref1 = ref1(:);

% Define the step positions and time durations
pos = [0, 3*pi/2, 3*pi/2, 4*pi/5]; % Step values
time_durations = [2, 1, 2]; % Duration for each range

% Define the range start times
start_times = [0, cumsum(time_durations)]; % Cumulative sum gives range boundaries

% Loop through each range and analyze step response characteristics
for i = 1:length(time_durations)
    % Define the time range
    t_start = start_times(i);
    t_end = start_times(i + 1);
    
    % Find indices within this time range
    idx_range = find((time1 >= t_start) & (time1 < t_end));
    
    % Check if valid data exists for this range
    if isempty(idx_range)
        fprintf('No data available for Range %d (%.2f to %.2f sec)\n', i, t_start, t_end);
        continue; % Skip this iteration
    end
    
    % Extract corresponding response and reference values
    t_range = time1(idx_range);
    response_range = response1(idx_range);
    ref_range = ref1(idx_range);
    
    % Determine if this is a step-up, step-down, or no change
    initial_ref = ref_range(1); % Initial value of the reference in this range
    target_ref = pos(i+1); % Target value for this step
    
    % Calculate step size from the reference signal
    step_size = abs(target_ref - initial_ref); % Magnitude of the step change
    
    % Check if there is no change in the reference signal
    if step_size == 0
        fprintf('No step change in Range %d (%.2f to %.2f sec). Skipping analysis.\n', i, t_start, t_end);
        continue; % Skip this iteration
    end
    
    if target_ref > initial_ref
        % Step-up: Use the original response and reference
        normalized_response = response_range;
        adjusted_ref = ref_range; % Reference remains unchanged
        ref_value = target_ref; % Use the target value as the reference
    else
        % Step-down: Normalize the response to 0 to step size
        normalized_response = (initial_ref - response_range) / step_size * step_size; % Scale to step size
        % Adjust the reference signal to match the normalized response
        adjusted_ref = (initial_ref - ref_range) / step_size * step_size;
        ref_value = step_size; % Use step size as the reference for normalized step-down
    end
    
    % Compute step response characteristics
    step_info = stepinfo(normalized_response, t_range, ref_value); 
    
    % Display step response characteristics
    fprintf('Step Response Characteristics for Range %d (%.2f to %.2f sec):\n', i, t_start, t_end);
    disp(step_info);
    
    % Plot the normalized response and adjusted reference
    figure;
    plot(t_range(:), normalized_response(:), 'r', 'LineWidth', 1.5); hold on;
    plot(t_range(:), adjusted_ref(:), 'b--', 'LineWidth', 1.5);  % Adjusted reference
    title(sprintf('Step Response: Range %d (%.2f to %.2f sec)', i, t_start, t_end));
    xlabel('Time (s)');
    ylabel('Response');
    legend('Normalized Response', 'Adjusted Reference');
    grid on;
end