% Extract time and response data
time1 = test2{2}.Values.Time;  % Time values
response1 = test2{2}.Values.Data;  % Measured response
ref1 = test2{6}.Values.Data;  % Reference signal

% Ensure data is column vectors
time1 = time1(:);
response1 = response1(:);
ref1 = ref1(:);

% Define the step positions and time durations
pos = [0, pi/2, pi, 3*pi/2]; % Step values
time_durations = [2, 3, 4]; % Duration for each range

% Define the range start times
start_times = [0, cumsum(time_durations)]; % Cumulative sum gives range boundaries

% Loop through each range and plot response and reference
for i = 1:length(time_durations)
    % Define the time range
    t_start = start_times(i);
    t_end = start_times(i + 1);
    
    % Find indices within this time range
    idx_range = find((time1 >= t_start) & (time1 < t_end));
    
    % Extract corresponding response and reference values
    t_range = time1(idx_range);
    response_range = response1(idx_range);
    ref_range = ref1(idx_range);
    
    % Determine the initial and final values for stepinfo()
    yinit = response_range(1); % Initial value of the response
    yfinal = ref_range(end); % Final value of the reference (target value)
    
    % Compute step-response characteristics using stepinfo(y, t, yfinal, yinit)
    step_info = stepinfo(response_range, t_range, yfinal, yinit);

    % Extract transient time
    transient_time = step_info.TransientTime;
    
    % Find the tracking error up to transient time
    idx_transient = (t_range - t_range(1)) <= transient_time; % Index before transient time
    tracking_error = abs(ref_range(idx_transient) - response_range(idx_transient));
    
    % Store tracking error for this step
    max_tracking_errors = max(tracking_error);
    
    % Display step-response characteristics
    fprintf('Step-Response Characteristics for Range %d (%.2f to %.2f sec):\n', i, t_start, t_end);
    disp(step_info);
    fprintf('Tracking Error: %d \n', max_tracking_errors);
    
    % Plot the response and reference for this range
    % figure;
    % plot(t_range, response_range, 'r', 'LineWidth', 1.5); hold on;
    % plot(t_range, ref_range, 'b--', 'LineWidth', 1.5);
    % title(sprintf('Response and Reference: Range %d (%.2f to %.2f sec)', i, t_start, t_end));
    % xlabel('Time (s)');
    % ylabel('Response / Reference');
    % legend('Response', 'Reference');
    % grid on;

end
