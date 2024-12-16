y = csvread('encoder.csv');
u = csvread('input_trans.csv');

L = length(y);
t = 0:0.001:(L-1)*0.001;

% figure(1);
% plot(t,u);
% grid on;

figure(2);
plot(t,y);
grid on;
%%
pidTuner()
%% PID Alminum
% Kp = 0.166, Ki = 0.42, Kd = 0.0163
%% PIDF 2022/03/10 200.0
% Kp = 20.5, Ki = 52.9, Kd = 1.75, Tf = 0.01
%% PIDF 2022/03/10 200.0
% Kp = 19.72, Ki = 52, Kd = 1.548, Tf = 0.0004297
