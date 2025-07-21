clear;

%% load log
data = csvread("current_data.csv");
% data = csvread("current_data2.csv");

data_battery = data(:,1);
data_cur_l = data(:,2);
data_cur_r = data(:,3);
data_duty_l = data(:,4);
data_duty_r = data(:,5);

n = length(data_battery);
ts = 0.02;
t = 0:1:n-1;
t = t'*ts;
%% plot data
close all;
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(3,2,[1 2]);
plot(t,data_battery,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('Battery [V]','Interpreter','latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,3);
plot(t,data_duty_l,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$d_l$ ','Interpreter','latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,4);
plot(t,data_cur_l,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$i_l$ [A]','Interpreter','latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,5);
plot(t,data_duty_r,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$d_r$ ','Interpreter','latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,6);
plot(t,data_cur_r,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$i_r$ [A]','Interpreter','latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);