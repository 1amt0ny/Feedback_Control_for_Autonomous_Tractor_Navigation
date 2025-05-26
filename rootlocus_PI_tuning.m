clear all;
clc;

% Define the plant constant
G_v = 0.2159;

% Define the range for proportional and integral gains
Kp_v = 0.1:0.5:5;  % Proportional gains for velocity
Ki_v = [0.5, 1, 1.5];  % Selected integral gains for velocity

% Initialize figure
figure;
hold on;

% Loop through Ki values to generate root locus for each
for Ki = Ki_v
    num_v = [G_v * Kp_v, G_v * Ki];  % Numerator of transfer function
    den_v = [1, G_v * Kp_v, G_v * Ki];  % Denominator of transfer function
    rlocus(tf(num_v, den_v));  % Plot root locus
end

title('Root Locus for Velocity Control with Selected K_i');
grid on;

% Define the plant constant
G_w = 0.238;

% Define the range for proportional and integral gains
Kp_w = 0.1:0.5:5;  % Proportional gains for turning rate
Ki_w = [0.5, 1, 1.5];  % Selected integral gains for turning rate

% Initialize figure
figure;
hold on;

% Loop through Ki values to generate root locus for each
for Ki = Ki_w
    num_w = [G_w * Kp_w, G_w * Ki];  % Numerator of transfer function
    den_w = [1, G_w * Kp_w, G_w * Ki];  % Denominator of transfer function
    rlocus(tf(num_w, den_w));  % Plot root locus
end

title('Root Locus for Turning Rate Control with Selected K_i');
grid on;
hold off;
