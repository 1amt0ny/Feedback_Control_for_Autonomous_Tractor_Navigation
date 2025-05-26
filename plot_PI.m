% Load data from the text file
opts = detectImportOptions('PI.txt', 'Delimiter', ' ', 'FileType', 'text');
preview('PI.txt', opts)

% If the preview shows the correct data structure, proceed with loading the data
data = readtable('PI.txt', 'Delimiter', ' ', 'ReadVariableNames', false);

% Display the first few rows of the data to check column assignments
disp(data(1:5, :));

% Open the file
fid = fopen('PI.txt', 'rt');

% Initialize arrays to store the data
time = [];
wRef = [];
wMes = [];
uRef = [];
uMes = [];

% Read the file line by line
while true
    line = fgetl(fid);
    if ~ischar(line)
        break;  % End of file
    end
    % Extract data from the line
    tokens = regexp(line, 'time: (\d+\.\d+) \[s\]wRef: ([-\d\.]+) \[rad/s\], wMes: ([-\d\.]+) \[rad/s\], uRef: ([-\d\.]+) \[m/s\], uMes: ([-\d\.]+) \[m/s\]', 'tokens');
    if isempty(tokens)
        continue; % Skip to the next line if no tokens found
    end
    tokens = tokens{1};
    time = [time; str2double(tokens{1})];
    wRef = [wRef; str2double(tokens{2})];
    wMes = [wMes; str2double(tokens{3})];
    uRef = [uRef; str2double(tokens{4})];
    uMes = [uMes; str2double(tokens{5})];
end

% Close the file
fclose(fid);

% Plotting wMes against wRef
figure;
plot(time, wRef, 'b', time, wMes, 'r--');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Reference vs. Measured Turning Rates');
legend('wRef', 'wMes');

% Plotting uMes against uRef
figure;
plot(time, uRef, 'b', time, uMes, 'r--');
xlabel('Time (s)');
ylabel('Linear Velocity (m/s)');
title('Reference vs. Measured Velocities');
legend('uRef', 'uMes');
