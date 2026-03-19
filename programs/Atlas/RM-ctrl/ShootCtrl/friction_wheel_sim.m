%% 摩擦轮发射机构 - 弹丸初速比例计算（完整版）
% 本脚本基于线性比例模型 v = k * v_wheel 估算弹丸离开发射机构时的初速，
% 其中 v_wheel 为摩擦轮线速度，k 为综合比例系数（需实验标定）。
% 可选：同时绘制基于物理参数的摩擦力做功模型作为对比。

clear; clc; close all;

%% ===================== 1. 用户输入参数 =====================
% 摩擦轮几何参数
R_wheel = 0.04;          % 摩擦轮半径 (m)   [典型值 0.03~0.05]
d_ball = 0.017;          % 弹丸直径 (m)     [17mm]

% 比例模型参数
k_linear = 0.8;          % 比例系数（通常 0.6~0.9，需实验标定）

% 电机转速范围
n_min = 1000;            % 最小转速 (rpm)
n_max = 8000;            % 最大转速 (rpm)
n_step = 100;            % 转速步长 (rpm)

% 是否同时绘制物理模型对比（0=仅比例模型，1=包含物理模型）
enable_physics = 1;

%% ===================== 2. 物理模型参数（仅当 enable_physics=1 时使用）=====================
if enable_physics
    gap = 0.0165;            % 两摩擦轮间距 (m) [略小于弹丸直径]
    delta = d_ball - gap;    % 过盈量 (m)
    mu = 0.5;                % 摩擦系数（PU与塑料弹丸）
    E_pu = 30e6;             % 摩擦轮材料弹性模量 (Pa)
    nu_pu = 0.45;            % 泊松比
    m_ball = 0.0032;         % 弹丸质量 (kg)
    eta = 0.8;               % 能量传递效率
end

%% ===================== 3. 计算 =====================
rpm = n_min:n_step:n_max;                % 转速数组
v_wheel = (2 * pi * rpm / 60) * R_wheel; % 摩擦轮线速度 (m/s)
v_ball_linear = k_linear * v_wheel;      % 比例模型弹丸初速 (m/s)

if enable_physics
    % 物理模型：摩擦力做功估算
    E_star = E_pu / (1 - nu_pu^2);
    N_avg = sqrt( (16 * R_wheel * E_star^2 * delta^3) / 9 );
    theta = acos(1 - delta / R_wheel);
    L_contact = R_wheel * 2 * theta;
    W_fric = mu * N_avg * L_contact;
    v_theory_min = sqrt(2 * eta * W_fric / m_ball);
    v_ball_physics = min(v_wheel, v_theory_min * ones(size(v_wheel)));
end

%% ===================== 4. 绘制曲线 =====================
figure('Position', [100, 100, 800, 500]);

% 主图：转速-初速曲线
plot(rpm, v_wheel, 'k:', 'LineWidth', 1.5, 'DisplayName', '摩擦轮线速度'); hold on;
plot(rpm, v_ball_linear, 'b-', 'LineWidth', 2, 'DisplayName', sprintf('比例模型 (k=%.2f)', k_linear));
if enable_physics
    plot(rpm, v_ball_physics, 'r--', 'LineWidth', 2, 'DisplayName', '物理模型（下限）');
end
xlabel('摩擦轮转速 (rpm)');
ylabel('弹丸初速 (m/s)');
title('摩擦轮转速 vs 弹丸出速');
legend('Location', 'northwest');
grid on;
xlim([n_min, n_max]);

% 在图上标注比例系数
text(0.1, 0.9, sprintf('比例系数 k = %.2f', k_linear), 'Units', 'normalized', ...
    'FontSize', 11, 'BackgroundColor', 'white');

%% ===================== 5. 命令行输出 =====================
fprintf('\n========== 比例模型计算结果 ==========\n');
fprintf('摩擦轮半径 R = %.3f m\n', R_wheel);
fprintf('比例系数 k = %.2f\n', k_linear);
fprintf('转速范围 %d ~ %d rpm\n', n_min, n_max);
fprintf('\n转速(rpm)\t线速度(m/s)\t比例模型初速(m/s)');
if enable_physics
    fprintf('\t物理模型初速(m/s)');
end
fprintf('\n');

% 每隔一定步长输出一行（例如每10个点）
step_display = 10;
for i = 1:step_display:length(rpm)
    fprintf('%d\t\t%.2f\t\t%.2f', rpm(i), v_wheel(i), v_ball_linear(i));
    if enable_physics
        fprintf('\t\t\t%.2f', v_ball_physics(i));
    end
    fprintf('\n');
end
fprintf('...\n');
fprintf('=======================================\n');

%% ===================== 6. （可选）保存数据到文件 =====================
% 如需保存数据，取消下一行的注释
% save('friction_wheel_data.mat', 'rpm', 'v_wheel', 'v_ball_linear');