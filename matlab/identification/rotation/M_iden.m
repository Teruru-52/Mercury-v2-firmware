% M系列信号によるシステム同定
%% M系列信号入力と応答
close all
clear

data = csvread('data.csv');

input = data(:, 1);
output = data(:, 2);

figure(1);
subplot(2,1,1);
plot(input,'LineWidth',2);
grid on
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$u_\omega$ [V]','Interpreter','latex','FontSize',20);
xlim([0 2540]);
ylim([-1.7 1.7]);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;

subplot(2,1,2);
plot(output,'LineWidth',2);
grid on;
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',20);
xlim([0 2540]);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;
%% システム同定のための準備
exp_input = input(1:1270,1);
val_input = input(1271:2540,1);
exp_output = output(1:1270,1);
val_output = output(1271:2540,1);
%% システム同定
systemIdentification
%% 2022/07/05
% Tp1 = 20.56
% Kp = 137.9
%%
pidTuner()