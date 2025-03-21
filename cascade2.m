% Extract time and response data
time1 = default_cascade{2}.Values.Time;  % Time values
response1 = default_cascade{2}.Values.Data;  % Measured response
ref1 = default_cascade{5}.Values.Data;  % Reference signal

% Ensure data is column vectors
time1 = time1(:);
response1 = response1(:);
ref1 = ref1(:);

% Define the specific time ranges
time_ranges = [0.67, 1.003; 2.333, 3.003; 5.002, 6.001; 7.334, 8.00; 8.666, 9.003];  % Time ranges in pairs

% Loop through each time range and plot response and reference
for i = 1:size(time_ranges, 1)
    % Define the start and end time for the current range
    t_start = time_ranges(i, 1);
    t_end = time_ranges(i, 2);
    
    % Find indices within this time range
    idx_range = find((time1 >= t_start) & (time1 < t_end));
    
    % Extract corresponding response and reference values
    t_range = time1(idx_range);
    response_range = response1(idx_range);
    ref_range = ref1(idx_range);
    
    
    tracking_error = abs(ref_range(t_start:t_end) - response_range(t_start:t_end));
    
    % Store maximum tracking error for this step
    max_tracking_errors = max(tracking_error);
    
    % Display step-response characteristics
    fprintf('Step-Response Characteristics for Range %d (%.2f to %.2f sec):\n', i, t_start, t_end);
    disp(step_info);
    fprintf('Tracking Error: %.4f \n', max_tracking_errors);
    
    % Plot the response and reference for this range
    % figure;
    % plot(t_range, response_range, 'r', 'LineWidth', 1.5); hold on;
    % plot(t_range, ref_range, 'b--', 'LineWidth', 1.5);
    % title(sprintf('Response and Reference: Range %d (%.2f to %.2f sec)', i, t_start, t_end));
    % xlabel('Time (s)');
    % ylabel('Response / Reference');
    % legend('Response', 'Reference');
end   
