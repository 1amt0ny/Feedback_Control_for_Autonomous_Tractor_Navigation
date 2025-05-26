% Define the plant constant for velocity control
G_v = 0.2159;

% Gain ratio
z = 3;  % Ki_v/Kp_v

% Define the transfer function component
s = tf('s');
L = (s + z) * G_v / s;

% Generate the root locus plot and use rlocfind to select K
figure;
rlocus(L);
title('Root Locus for Velocity Control');
grid on;

% Use rlocfind to interactively select K
[K, poles] = rlocfind(L);

% Calculate Kp and Ki based on selected K
Kp_v = K;
Ki_v = 3 * K;

% Display chosen gains
fprintf('Selected Kp: %.3f\n', Kp_v);
fprintf('Selected Ki: %.3f\n', Ki_v);

% For turning rate control, similar steps are repeated
% Define the plant constant for turning rate control
G_w = 0.238;

% Define the transfer function component for turning rate
L_w = (s + z) * G_w / s;

% Generate the root locus plot and use rlocfind to select K
figure;
rlocus(L_w);
title('Root Locus for Turning Rate Control');
grid on;

% Use rlocfind to interactively select K for turning rate
[K_w, poles_w] = rlocfind(L_w);

% Calculate Kp and Ki based on selected K for turning rate
Kp_w = K_w;
Ki_w = 3 * K_w;

% Display chosen gains for turning rate
fprintf('Selected Kp for turning rate: %.3f\n', Kp_w);
fprintf('Selected Ki for turning rate: %.3f\n', Ki_w);
