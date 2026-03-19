%% RoboMaster 2026 多发射模式热量控制仿真（30Hz上限+缓冲热量）
clear; clc; close all;

%% ===================== 1. 核心参数配置 =====================
% 规则固定参数（步兵17mm）
heat_per_shot = 10;       % 单发射热（点/发）
cool_freq = 10;           % 冷却结算频率（Hz）
dt = 1/cool_freq;         % 仿真步长（0.1秒）
sim_time = 60;            % 总仿真时间（秒）

% 发射机构类型：1=爆发优先 2=冷却优先
type = 1;
if type == 1
    Q0 = 230;             % 热量上限
    cool_rate = 14;       % 冷却速率（点/秒）
    buffer_heat = 30;     % 缓冲热量（预留冷却空间）
else
    Q0 = 88;
    cool_rate = 24;
    buffer_heat = 20;
end
safe_Q = Q0 - buffer_heat; % 安全上限（绝对不超）

% 发射模式配置（模式1=点射 2=双联射 3=连射）
% 模式切换时间：第10秒点射→第20秒双联射→第30秒连射→第40秒点射
mode_switch_time = [10, 20, 30, 40];
mode_list = [1, 2, 3, 1]; % 对应切换的模式
% 各模式射速上限（最高30Hz）
mode_speed_limit = [5, 10, 30]; % 1=点射5Hz 2=双联射10Hz 3=连射30Hz

%% ===================== 2. 初始化变量 =====================
time_list = 0:dt:(sim_time-dt);
heat_list = zeros(1, length(time_list));    % 实时热量
shot_mode_list = zeros(1, length(time_list));% 实时发射模式
shot_freq_list = zeros(1, length(time_list));% 实时射速
shot_num_list = zeros(1, length(time_list)); % 每步发射数
current_mode = 1; % 初始模式：点射

%% ===================== 3. 核心仿真循环 =====================
for i = 1:length(time_list)
    current_time = time_list(i);
    current_heat = heat_list(i);
    
    % -------------------- 3.1 模式切换（按时间） --------------------
    for j = 1:length(mode_switch_time)
        if current_time >= mode_switch_time(j)
            current_mode = mode_list(j);
        end
    end
    shot_mode_list(i) = current_mode;
    
    % -------------------- 3.2 计算当前模式下的最大可发射数 --------------------
    % 单模式产热量（点/次触发）
    if current_mode == 1 % 点射：1发/次
        single_mode_heat = heat_per_shot;
    elseif current_mode == 2 % 双联射：2发/次
        single_mode_heat = 2*heat_per_shot;
    else % 连射：1发/次，持续连发
        single_mode_heat = heat_per_shot;
    end
    
    % 最大可发射数（不超安全上限）
    max_shot_num = floor((safe_Q - current_heat) / single_mode_heat);
    max_shot_num = max(max_shot_num, 0); % 最少0发
    
    % -------------------- 3.3 计算当前模式下的射速（不超模式上限） --------------------
    mode_speed = mode_speed_limit(current_mode);
    % 每步（0.1秒）最大发射数 = 模式射速 × 步长
    max_shot_per_step = mode_speed * dt;
    % 实际发射数（取整，且不超最大可发射数）
    shot_num = min(round(max_shot_per_step), max_shot_num);
    shot_num_list(i) = shot_num;
    
    % 实际射速（发/秒）
    real_freq = shot_num / dt;
    shot_freq_list(i) = real_freq;
    
    % -------------------- 3.4 热量更新（产热+冷却） --------------------
    % 1. 发弹产热
    heat_produce = shot_num * heat_per_shot;
    % 2. 本步冷却
    heat_cool = cool_rate * dt;
    % 3. 下一步热量（≥0）
    if i < length(time_list)
        heat_list(i+1) = max(current_heat + heat_produce - heat_cool, 0);
    end
end

%% ===================== 4. 结果可视化 =====================
figure('Position', [100, 100, 1200, 900]);

% 子图1：热量变化（含安全上限/缓冲线）
subplot(3,1,1);
plot(time_list, heat_list, 'b-', 'LineWidth', 1.5);
hold on;
plot(time_list, ones(1,length(time_list))*Q0, 'r--', 'LineWidth', 1.2);
plot(time_list, ones(1,length(time_list))*safe_Q, 'g--', 'LineWidth', 1.2);
xlabel('时间（秒）');
ylabel('热量值（点）');
% 标题（替换三元运算符）
if type == 1
    title('爆发优先 多模式热量控制（缓冲热量=30点）');
else
    title('冷却优先 多模式热量控制（缓冲热量=20点）');
end
legend('实时热量','热量上限Q0','安全上限（Q0-缓冲）','Location','best');
grid on;

% 子图2：发射模式变化
subplot(3,1,2);
plot(time_list, shot_mode_list, 'k-', 'LineWidth', 1.5);
xlabel('时间（秒）');
ylabel('发射模式（1=点射 2=双联射 3=连射）');
title('发射模式切换');
grid on;

% 子图3：实时射速（最高30Hz）—— 修复颜色参数为全版本兼容的'm--'
subplot(3,1,3);
plot(time_list, shot_freq_list, 'm-', 'LineWidth', 1.5);
hold on;
plot(time_list, ones(1,length(time_list))*30, 'm--', 'LineWidth', 1.2); % 替换orange--为m--
xlabel('时间（秒）');
ylabel('射速（发/秒）');
title('实时射速（最高30Hz）');
legend('实际射速','30Hz上限','Location','best');
grid on;

%% ===================== 5. 统计结果（移除所有三元运算符） =====================
total_shot = sum(shot_num_list);
avg_freq = total_shot / sim_time;
max_heat = max(heat_list);

fprintf('===== 仿真结果 =====\n');
% 1. 发射机构类型
if type == 1
    fprintf('发射机构类型：爆发优先\n');
else
    fprintf('发射机构类型：冷却优先\n');
end
% 2. 基础统计
fprintf('总发弹数：%d发\n', total_shot);
fprintf('平均射速：%.2f发/秒\n', avg_freq);
fprintf('最大热量：%.1f点（安全上限%d点）\n', max_heat, safe_Q);
% 3. 是否超限判断
if max_heat > Q0
    fprintf('是否超限：是\n');
else
    fprintf('是否超限：否\n');
end