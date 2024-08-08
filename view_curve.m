close all
clear
clc

[odom_data_x, odom_data_y, odom_data_yaw, l_acc_x, l_acc_y, l_acc_z, a_acc_x, a_acc_y, a_acc_z, imu_data_yaw, est_x, est_y, est_yaw] = importfile("/home/lio/robot_system/data.log", [50, 30000]);

% 绘制encoder pos
figure
subplot(3, 1, 1)
plot(odom_data_x, odom_data_y, 'Color', 'r', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Encoder sensor position', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold')
grid on
hold on

subplot(3, 1, 2)
plot(odom_data_x, 'Color', 'r', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Encoder sensor position X', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold')
grid on
hold on

subplot(3, 1, 3)
plot(odom_data_y, 'Color', 'r', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Encoder sensor position Y', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold')
grid on
hold on

% 绘制所有数据的曲线
figure
subplot(3,3,1)
plot(odom_data_yaw, 'Color', 'r', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Encoder sensor Yaw', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Yaw (rad)', 'FontSize', 12, 'FontWeight', 'bold')
grid on
hold on

mean_value = mean(odom_data_yaw);
std_value = std(odom_data_yaw);
[max_value, max_index] = max(odom_data_yaw);
[min_value, min_index] = min(odom_data_yaw);

plot(max_index, max_value, 'go', 'LineWidth', 2);
plot(min_index, min_value, 'bo', 'LineWidth', 2);
text(max_index, max_value, sprintf('Max: %.6f', max_value), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')
text(min_index, min_value, sprintf('Min: %.6f', min_value), 'FontSize', 12, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right')
yline(mean_value, 'k--', 'LineWidth', 1.5);
text(length(odom_data_yaw), mean_value, sprintf('Mean: %.6f', mean_value), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')

legend({
    sprintf('Yaw of odom data (Variance: %.6f)', std_value), ...
    sprintf('Max: %.6f', max_value), ...
    sprintf('Min: %.6f', min_value), ...
    sprintf('Mean: %.6f', mean_value)
}, 'FontSize', 12)
hold off

subplot(3,3,4)
plot(est_yaw, 'Color', 'b', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Estimate Yaw', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Yaw (rad)', 'FontSize', 12, 'FontWeight', 'bold')
grid on
hold on

mean_value = mean(est_yaw);
std_value = std(est_yaw);
[max_value, max_index] = max(est_yaw);
[min_value, min_index] = min(est_yaw);

plot(max_index, max_value, 'go', 'LineWidth', 2);
plot(min_index, min_value, 'bo', 'LineWidth', 2);
text(max_index, max_value, sprintf('Max: %.6f', max_value), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')
text(min_index, min_value, sprintf('Min: %.6f', min_value), 'FontSize', 12, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right')
yline(mean_value, 'k--', 'LineWidth', 1.5);
text(length(est_yaw), mean_value, sprintf('Mean: %.6f', mean_value), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')

legend({
    sprintf('Yaw of estimate (Variance: %.6f)', std_value), ...
    sprintf('Max: %.6f', max_value), ...
    sprintf('Min: %.6f', min_value), ...
    sprintf('Mean: %.6f', mean_value)
}, 'FontSize', 12)
hold off

subplot(3,3,7)
plot(imu_data_yaw, 'Color', 'g', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Imu Sensor Yaw', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Yaw (rad)', 'FontSize', 12, 'FontWeight', 'bold')
grid on
hold on

mean_value = mean(imu_data_yaw);
std_value = std(imu_data_yaw);
[max_value, max_index] = max(imu_data_yaw);
[min_value, min_index] = min(imu_data_yaw);

plot(max_index, max_value, 'go', 'LineWidth', 2);
plot(min_index, min_value, 'bo', 'LineWidth', 2);
text(max_index, max_value, sprintf('Max: %.6f', max_value), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')
text(min_index, min_value, sprintf('Min: %.6f', min_value), 'FontSize', 12, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right')
yline(mean_value, 'k--', 'LineWidth', 1.5);
text(length(est_yaw), mean_value, sprintf('Mean: %.6f', mean_value), 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')

legend({
    sprintf('Yaw of imu sensor (Variance: %.6f)', std_value), ...
    sprintf('Max: %.6f', max_value), ...
    sprintf('Min: %.6f', min_value), ...
    sprintf('Mean: %.6f', mean_value)
}, 'FontSize', 12)
hold off

% 绘制加速度数据
subplot(3,3,2)
plot(a_acc_x, 'Color', 'r', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Angular Acceleration X', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Acceleration (rad/s^2)', 'FontSize', 12, 'FontWeight', 'bold')
grid on

subplot(3,3,5)
plot(a_acc_y, 'Color', 'g', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Angular Acceleration Y', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Acceleration (rad/s^2)', 'FontSize', 12, 'FontWeight', 'bold')
grid on

subplot(3,3,8)
plot(a_acc_z, 'Color', 'b', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Angular Acceleration Z', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Acceleration (rad/s^2)', 'FontSize', 12, 'FontWeight', 'bold')
grid on

% 绘制线性加速度数据
subplot(3,3,3)
plot(l_acc_x, 'Color', 'r', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Linear Acceleration X', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Acceleration (m/s^2)', 'FontSize', 12, 'FontWeight', 'bold')
grid on

subplot(3,3,6)
plot(l_acc_y, 'Color', 'g', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Linear Acceleration Y', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Acceleration (m/s^2)', 'FontSize', 12, 'FontWeight', 'bold')
grid on

subplot(3,3,9)
plot(l_acc_z, 'Color', 'b', 'Marker', 'o', 'MarkerSize', 6, 'LineWidth', 1.5)
title('Linear Acceleration Z', 'FontSize', 16, 'FontWeight', 'bold')
xlabel('Time(x20ms)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Acceleration (m/s^2)', 'FontSize', 12, 'FontWeight', 'bold')
grid on

function [odom_data_x, odom_data_y, odom_data_yaw, l_acc_x, l_acc_y, l_acc_z, a_acc_x, a_acc_y, a_acc_z, imu_data_yaw, est_x, est_y, est_yaw] = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  [ODOM_DATA_X, ODOM_DATA_Y, ODOM_DATA_YAW, L_ACC_X, L_ACC_Y, L_ACC_Z,
%  A_ACC_X, A_ACC_Y, A_ACC_Z, IMU_DATA_YAW, EST_X, EST_Y, EST_YAW] =
%  IMPORTFILE(FILENAME) reads data from text file FILENAME for the
%  default selection.  Returns the data as column vectors.
%
%  [ODOM_DATA_X, ODOM_DATA_Y, ODOM_DATA_YAW, L_ACC_X, L_ACC_Y, L_ACC_Z,
%  A_ACC_X, A_ACC_Y, A_ACC_Z, IMU_DATA_YAW, EST_X, EST_Y, EST_YAW] =
%  IMPORTFILE(FILE, DATALINES) reads data for the specified row
%  interval(s) of text file FILENAME. Specify DATALINES as a positive
%  scalar integer or a N-by-2 array of positive scalar integers for
%  dis-contiguous row intervals.
%
%  Example:
%  [odom_data_x, odom_data_y, odom_data_yaw, l_acc_x, l_acc_y, l_acc_z, a_acc_x, a_acc_y, a_acc_z, imu_data_yaw, est_x, est_y, est_yaw] = importfile("/home/lio/robot_system/data.log", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 07-Aug-2024 14:36:12

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 33);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = [" ", ",", "[", "]"];

% Specify column names and types
opts.VariableNames = ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "odom_data_x", "Var8", "odom_data_y", "Var10", "odom_data_yaw", "Var12", "Var13", "l_acc_x", "Var15", "l_acc_y", "Var17", "l_acc_z", "Var19", "a_acc_x", "Var21", "a_acc_y", "Var23", "a_acc_z", "Var25", "imu_data_yaw", "Var27", "Var28", "est_x", "Var30", "est_y", "Var32", "est_yaw"];
opts.SelectedVariableNames = ["odom_data_x", "odom_data_y", "odom_data_yaw", "l_acc_x", "l_acc_y", "l_acc_z", "a_acc_x", "a_acc_y", "a_acc_z", "imu_data_yaw", "est_x", "est_y", "est_yaw"];
opts.VariableTypes = ["string", "string", "string", "string", "string", "string", "double", "string", "double", "string", "double", "string", "string", "double", "string", "double", "string", "double", "string", "double", "string", "double", "string", "double", "string", "double", "string", "string", "double", "string", "double", "string", "double"];

% Specify file level properties
opts.ImportErrorRule = "omitrow";
opts.MissingRule = "omitrow";
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";

% Specify variable properties
opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var8", "Var10", "Var12", "Var13", "Var15", "Var17", "Var19", "Var21", "Var23", "Var25", "Var27", "Var28", "Var30", "Var32"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var8", "Var10", "Var12", "Var13", "Var15", "Var17", "Var19", "Var21", "Var23", "Var25", "Var27", "Var28", "Var30", "Var32"], "EmptyFieldRule", "auto");
opts = setvaropts(opts, ["odom_data_x", "odom_data_y", "odom_data_yaw", "l_acc_x", "l_acc_y", "l_acc_z", "a_acc_x", "a_acc_y", "a_acc_z", "imu_data_yaw", "est_x", "est_y", "est_yaw"], "ThousandsSeparator", ",");

% Import the data
tbl = readtable(filename, opts);

%% Convert to output type
odom_data_x = tbl.odom_data_x;
odom_data_y = tbl.odom_data_y;
odom_data_yaw = tbl.odom_data_yaw;
l_acc_x = tbl.l_acc_x;
l_acc_y = tbl.l_acc_y;
l_acc_z = tbl.l_acc_z;
a_acc_x = tbl.a_acc_x;
a_acc_y = tbl.a_acc_y;
a_acc_z = tbl.a_acc_z;
imu_data_yaw = tbl.imu_data_yaw;
est_x = tbl.est_x;
est_y = tbl.est_y;
est_yaw = tbl.est_yaw;
end