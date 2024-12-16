%%
y = csvread('output.csv');
u = csvread('input.csv');

L = length(y);
t = 0:0.001:(L-1)*0.001;

figure(1);
plot(t,u);
grid on;

figure(2);
plot(t,y);
grid on;
%%
pidTuner()
%% PIDF 1.5V
%  Kp = 3.71, Ki = 183, Kd = 0.00333
sys1 = tf([0 1], [1 0]);
% sysd1 = c2d(sys1,0.001,'tustin')
sysd1 = c2d(sys1,0.001,'zoh')
Tf = 3.1e-05;
sys2 = tf([0 1], [Tf 1])
% sysd2 = c2d(sys2,0.001,'tustin')
sysd2 = c2d(sys2,0.001,'zoh')
%% PIDF 2.0V
%  Kp = 2.15, Ki = 9.81, Kd = 0.104
sys1 = tf([0 1], [1 0]);
sysd1 = c2d(sys1,0.001,'zoh')
Tf = 0.00572;
sys2 = tf([0 1], [Tf 1]);
sysd2 = c2d(sys2,0.001,'zoh')
%% PID 2.0V
% Kp = 2.13, Ki = 9.82, Kd = 0.115
sys1 = tf([0 1], [1 0]);
sysd1 = c2d(sys1,0.001,'zoh')
%% PIDF 2022/03/10 2.0V
% Kp = 3.75, Ki = 15.7, Kd = 0.213
Tf = 0.00621;
sys2 = tf([1 0], [Tf 1]);
sysd2 = c2d(sys2,0.01,'zoh')
%% PID
% Kp = 3.68, Ki = 186, Kd = 0.00446
sys1 = tf([0 1], [1 0]);
sysd1 = c2d(sys1,0.001,'zoh')
% sys2 = tf([1 0], [0 1])
% sysd2 = c2d(sys2,0.001,'zoh')
%% PIDF 2022/03/10 300.0
% Kp = 948, Ki = 2240, Kd = 77.3
Tf = 0.0103;
% sys2 = tf([1 0], [Tf 1]);
% sysd2 = c2d(sys2,0.01,'zoh')
%% PIDF 2022/03/10 300.0
Kp = 18.2;
Ki = 332;
Kd = -0.072;
Tf = 0.0289;
G = tf([0.11081], [3.2263 3.10658 1])
Fp = Kp * tf([1 0],[0 1]);
Fi = Ki * tf([0 1], [1 0]);
Fd = Kd * tf([1 0], [Tf 1]);
Fpi = series(Fp, Fi);
Fpid = series(Fpi, Fd)
K = Fpid / G
%% PIDF 2022/04/17 2.0V
Kp = 0.734;
Ki = 17.1;
Kd = -0.00185;
Tf = 0.012;
Tp1 = 0.095935;
G = tf([Kp], [Tp1 1])

%% PIDF 2022/04/17 3.0V
Kp = 0.462;
Ki = 7.78;
Kd = -0.00218;
Tf = 0.0139;
Tp1 = 0.095935;
G = tf([11.59], [Tp1 1])