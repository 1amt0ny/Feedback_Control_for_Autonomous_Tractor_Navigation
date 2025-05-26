clc;
clear all;

% Define the plant constant
G_v = 0.2159;

% Selected gains
Kp_v = 0.5;
Ki_v = 1.5;

% This is incorrect
% Define the transfer function for velocity control
%num_v = [G_v * Kp_v, G_v * Ki_v];
%den_v = [(1 + G_v * Kp_v), G_v * Ki_v];
%--------------------------------------

% The loop gain (Kp+Ki/s)Gv should be rewritten 
% as Kp(s+Ki/Kp)Gv/s. Then you choose Ki/Kp while 
% Kp serves as positive K from the rootlocus method
% (this is ECE141 material, please review it).
% Therefore, for the root locus we use
z=Ki_v/Kp_v;
s=tf('s')
L=(s+z)*G_v/s

figure
rlocus(L)
title('Root Locus for Velocity Control');
grid on;

% Since the sketch of the root locus will  show that 
% there is only single branch to the root locus departing (K=0) 
% from the pole at the origin and arriving (K=infty) to the zero 
% at Ki_v/Kp_v, it is important to show that the system pole 
% is  about 10 or 6 is smaller than 1/Ts=(1/50ms)=20
% The value of Ki_v/Kp_v=3 is a safe choice since 3 < 20/6,
% yet you have to place the pole by selecting K. Once you do it, then 
% Kp_v=K and Ki_v = 3K.   You can compute where is the closed loop 
% pole for the selected K. The simlar process should be repeated for 
% the turning rate controller.


% Incorrect
% Generate the root locus plot for velocity control
%figure;
%rlocus(tf(num_v, den_v));
%title('Root Locus for Velocity Control with Selected Gains');
%grid on;

% Define the plant constant
G_w = 0.238;

% Selected gains
Kp_w = 0.3;
Ki_w = 1.0;

% Incorrect
% Define the transfer function for turning rate control
%num_w = [G_w * Kp_w, G_w * Ki_w];
%den_w = [(1 + G_w * Kp_w), G_w * Ki_w];

z=Ki_w/Kp_w;
s=tf('s')
L=(s+z)*G_w/s

figure
rlocus(L)
title('Root Locus for Turning Rate Control');
grid on;

% Incorrect
% Generate the root locus plot for turning rate control
%figure;
%rlocus(tf(num_w, den_w));
%title('Root Locus for Turning Rate Control with Selected Gains');
%grid on;


% Calculate closed-loop poles:

% For Velocity Control
den_v = [1 + G_v*Kp_v, G_v*Ki_v];
poles_velocity = roots(den_v);

% For Turning Rate Control
den_w = [1 + G_w*Kp_w, G_w*Ki_w];
poles_turning_rate = roots(den_w);

% Display the poles
disp('Poles for Velocity Control:');
disp(poles_velocity);

disp('Poles for Turning Rate Control:');
disp(poles_turning_rate);
