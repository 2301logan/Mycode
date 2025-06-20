clc;
clear;

%% === 系统参数 ===
K = 9.8386;                   % 系统增益
tau = 178.5182;              % 延迟时间
T = 2859.0614;               % 时间常数
step_amplitude = 3.5;        % 阶跃输入幅值
setpoint = 35;               % 设定值
initial_temp = 16.8;         % 初始温度

%% === 建立系统模型（Pade近似） ===
s = tf('s');
G = K * exp(-tau * s) / (T * s + 1);
G_pade = pade(G, 1);  % 一阶Pade近似

%% === 目标函数定义 ===
pid_objective = @(pid_param) ...
    compute_cost(pid_param, G_pade, setpoint, initial_temp);

%% === 粒子群优化PSO ===
nvars = 3;                       % [Kp, Ki, Kd]
lb = [0, 0, 0];                  % 下界
ub = [100, 10, 10];              % 上界
opts = optimoptions('particleswarm', ...
    'Display', 'iter', ...
    'SwarmSize', 30, ...
    'MaxIterations', 50);
[opt_pid, fval] = particleswarm(pid_objective, nvars, lb, ub, opts);

%% === 最优PID控制器 ===
Kp = opt_pid(1);
Ki = opt_pid(2);
Kd = opt_pid(3);
C_opt = pid(Kp, Ki, Kd);

%% === 闭环仿真 ===
sys_cl = feedback(C_opt * G_pade, 1);
t = 0:1:15000;
[y, t_out] = step(setpoint * sys_cl, t);
y = y + initial_temp - y(1);  % 调整初始温度

%% === 绘图输出 ===
figure;
plot(t_out, y, 'b', 'LineWidth', 1.5);
hold on;
yline(setpoint, '--r', '设定值');
xlabel('时间 (秒)');
ylabel('温度 (°C)');
title('闭环系统阶跃响应');
legend('响应曲线', '设定值');
grid on;

%% === 动态性能指标分析 ===
info = stepinfo(y, t_out, setpoint);

% 输出中文的动态性能指标
fprintf('\n动态性能指标:\n');
fprintf('上升时间: %.4f 秒\n', info.RiseTime);
fprintf('暂态时间: %.4f 秒\n', info.TransientTime);
fprintf('稳态时间: %.4f 秒\n', info.SettlingTime);
fprintf('稳态最小值: %.4f °C\n', info.SettlingMin);
fprintf('稳态最大值: %.4f °C\n', info.SettlingMax);
fprintf('超调: %.2f %%\n', info.Overshoot);
fprintf('下冲: %.2f %%\n', info.Undershoot);
fprintf('峰值: %.4f °C\n', info.Peak);
fprintf('峰值时间: %.4f 秒\n', info.PeakTime);

%% 输出最优PID参数
fprintf('\n最优PID参数: Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp, Ki, Kd);

%% === 子函数: 成本函数（ITAE） ===
function cost = compute_cost(pid_param, G, setpoint, initial_temp)
    % 提取PID参数
    Kp = pid_param(1);
    Ki = pid_param(2);
    Kd = pid_param(3);

    % 创建PID控制器
    C = pid(Kp, Ki, Kd);
    
    % 计算闭环系统
    sys_cl = feedback(C * G, 1);

    % 仿真时间
    t = 0:1:10000;
    [y, t_out] = step(setpoint * sys_cl, t);
    y = y + initial_temp - y(1);  % 调整初始温度偏移

    % 检查是否有NaN或Inf值
    if any(isnan(y)) || any(isinf(y))
        cost = 1e10;  % 返回一个较大的惩罚值，确保目标函数稳定
        return;
    end

    % 计算误差
    error = abs(setpoint - y);

    % 确保误差和时间向量是列向量
    t = t(:);
    error = error(:);

    % 检查误差是否有NaN或Inf
    if any(isnan(error)) || any(isinf(error))
        cost = 1e10;  % 如果误差中有无效值，返回惩罚值
        return;
    end

    % 计算ITAE：积分时间加权绝对误差，返回标量
    cost = sum(t .* error);  % 使用sum确保返回标量
end
