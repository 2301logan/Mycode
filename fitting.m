% 读取并解析CSV格式的温度响应数据
data = fileread('temperature.csv');        % 读取整个文件内容
lines = strsplit(data, '\n');              % 按行分割文本数据

% 初始化时间和温度数据存储数组
time = [];
temperature = [];

% 遍历每一行数据（从第2行开始，跳过标题行）
for i = 2:length(lines)
    if isempty(lines{i}), continue; end    % 跳过空行
    parts = strsplit(strtrim(lines{i}), ','); % 按逗号分割每行数据
    time(end+1) = str2double(parts{1});    % 存储时间数据
    temperature(end+1) = str2double(parts{2}); % 存储温度数据
end

% 数据平滑处理（使用移动平均）
smoothed_temperature = movmean(temperature, 10); % 10点窗口的移动平均

% 计算系统特性参数（使用平滑后的数据）
y0 = mean(smoothed_temperature(1:10));              % 初始值：前10个数据点的平均值
y_inf = mean(smoothed_temperature(end-99:end));     % 稳态值：最后100个数据点的平均值
Delta_y = y_inf - y0;                               % 总变化量
K = Delta_y / 3.5;                                  % 系统增益（假设阶跃输入幅值为3.5）

% 计算40%和80%响应点对应的温度值（用于拟合一阶滞后模型）
y40 = y0 + 0.4 * Delta_y;  % 40%响应点温度
y80 = y0 + 0.8 * Delta_y;  % 80%响应点温度

% 确定达到关键点的时间
idx40 = find(smoothed_temperature > y40, 1);        % 首次超过40%响应的索引
t40 = time(idx40);                         % 对应时间

idx80 = find(smoothed_temperature > y80, 1);        % 首次超过80%响应的索引
t80 = time(idx80);                         % 对应时间

fprintf('40%%响应时间 t40 = %.4f 秒\n', t40);
fprintf('80%%响应时间 t80 = %.4f 秒\n', t80);

% 基于40%-80%响应法计算模型参数
T = (t80 - t40) / (log(5) - log(5/3));     % 时间常数计算
tau = t40 - T * log(5/3);                  % 滞后时间计算

% 构建一阶滞后加纯延迟(FOOPDT)传递函数模型
s = tf('s');                               % 定义拉普拉斯变量
sys = K * exp(-tau*s) / (T*s + 1);         % 构建传递函数模型

% 输出模型参数结果
fprintf('系统传递函数: G(s) = %.4f * e^(-%.4fs) / (%.4fs + 1)\n', K, tau, T);
fprintf('参数解析:\n');
fprintf('增益 K = %.4f\n', K);             % 系统稳态增益
fprintf('滞后时间 τ = %.4f s\n', tau);     % 纯延迟时间
fprintf('时间常数 T = %.4f s\n', T);       % 系统响应速度指标

% 可视化分析结果
figure;

% 上半部分：原始数据与关键点分析
subplot(2,1,1);
plot(time, temperature, 'b', 'LineWidth', 1.5); % 绘制原始温度数据
hold on;
plot([time(1), time(end)], [y_inf, y_inf], 'r--'); % 稳态值水平线
plot([time(1), time(end)], [y40, y40], 'g--');     % 40%响应水平线
plot([time(1), time(end)], [y80, y80], 'm--');     % 80%响应水平线
plot(t40, y40, 'ro', 'MarkerSize', 8);            % 标记40%响应点
plot(t80, y80, 'mo', 'MarkerSize', 8);            % 标记80%响应点
legend('实测数据', '稳态值', '40%响应线', '80%响应线', '40%响应点', '80%响应点');
title('阶跃响应数据与关键点');
xlabel('时间 (s)');
ylabel('温度');
grid on;

% 下半部分：模型验证（对比实测数据与模型预测）
subplot(2,1,2);
[sim_y, sim_t] = step(3.5*sys, time(end));     % 生成模型的3.5阶跃响应;还可以用ones(size(time))
plot(time, smoothed_temperature, 'b', 'LineWidth', 1.5); % 绘制平滑后的数据
hold on;
plot(sim_t, sim_y + y0, 'r--', 'LineWidth', 1.5); % 绘制模型预测结果（添加初始偏移）
legend('平滑后数据', '模型拟合');
title('模型验证');
xlabel('时间 (s)');
ylabel('温度');
grid on;

% 平滑前后数据对比
figure;
subplot(2,1,1);
plot(time, temperature, 'b', 'LineWidth', 1.5); % 绘制平滑前的温度数据
title('平滑前的温度数据');
xlabel('时间 (s)');
ylabel('温度');
grid on;

subplot(2,1,2);
plot(time, smoothed_temperature, 'g', 'LineWidth', 1.5); % 绘制平滑后的温度数据
title('平滑后的温度数据');
xlabel('时间 (s)');
ylabel('温度');
grid on;

% 设置坐标轴范围一致
% 获取最大和最小的时间值和温度值
time_min = min(time);
time_max = max(time);
temp_min = min([temperature, smoothed_temperature]);
temp_max = max([temperature, smoothed_temperature]);

% 设置坐标轴范围
subplot(2,1,1);
xlim([time_min, time_max]);      % 统一时间轴范围
ylim([temp_min - 1, temp_max + 1]);  % 统一温度轴范围

subplot(2,1,2);
xlim([time_min, time_max]);      % 统一时间轴范围
ylim([temp_min - 1, temp_max + 1]);  % 统一温度轴范围
