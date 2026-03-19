%% RM 2026 底盘 LQR+卡尔曼控制 仿真主程序（类实现）
clear; clc; close all;

%% ===================== 1. 配置参数 =====================
DT = 0.001;               % 控制周期1ms
T_END = 15;               % 仿真时长15s
T = 0:DT:T_END;           % 时间轴
N = length(T);            % 总步数
season_type = '2026';     % 赛季类型（'2025'/'2026'）
P_limit_type = 1;         % 1=血量优先 2=功率优先

%% ===================== 2. 初始化控制器 =====================
chassis = ChassisController(DT, season_type, P_limit_type);

%% ===================== 3. 初始化仿真变量 =====================
% 速度指令
Vx_ref = zeros(1, N);
Vy_ref = zeros(1, N);
Vturn_ref = zeros(1, N);
% 输出状态记录
w_ref_log = zeros(4, N);
w_actual_log = zeros(4, N);
I_actual_log = zeros(4, N);
P_meas_log = zeros(1, N);
P_est_log = zeros(1, N);
P_pred_log = zeros(1, N);
speed_scale_log = zeros(1, N);
Z_log = zeros(1, N);
warn_flag_log = zeros(1, N);

%% ===================== 4. 生成速度指令工况 =====================
for k = 1:N
    t = T(k);
    if t < 1          % 0~1s：静止
        Vx_ref(k) = 0;
        Vy_ref(k) = 0;
        Vturn_ref(k) = 0;
    elseif t >= 1 && t < 2  % 1~2s：突发急加速
        Vx_ref(k) = (t-1)*5;
        Vy_ref(k) = 0;
        Vturn_ref(k) = 0.8*(t-1);
    elseif t >= 2 && t < 5  % 2~5s：持续高功率
        Vx_ref(k) = 3;
        Vy_ref(k) = 0.5;
        Vturn_ref(k) = 0.8;
    elseif t >= 5 && t < 8  % 5~8s：低功率恢复
        Vx_ref(k) = 0.5;
        Vy_ref(k) = 0.2;
        Vturn_ref(k) = 0.4;
    elseif t >= 8 && t < 10 % 8~10s：二次突发加速
        Vx_ref(k) = 0.5 + (t-8)*4;
        Vy_ref(k) = 0.2 + (t-8)*0.3;
        Vturn_ref(k) = 0.4 + (t-8)*0.4;
    else                    % 10~15s：匀速
        Vx_ref(k) = 2;
        Vy_ref(k) = 0.5;
        Vturn_ref(k) = 0.6;
    end
end

%% ===================== 5. 核心仿真循环 =====================
for k = 1:N
    t = T(k);
    % 底盘控制
    chassis.control(Vx_ref(k), Vy_ref(k), Vturn_ref(k), t);
    
    % 记录状态（所有属性均可正常访问）
    w_ref_log(:, k) = chassis.w_ref;
    w_actual_log(:, k) = chassis.w_actual;
    I_actual_log(:, k) = chassis.I_actual;
    P_meas_log(k) = chassis.P_meas;
    P_est_log(k) = chassis.kf_estimator.P_est;
    P_pred_log(k) = chassis.kf_estimator.P_pred;
    speed_scale_log(k) = chassis.speed_scale;
    Z_log(k) = chassis.rule_manager.Z;
    warn_flag_log(k) = chassis.rule_manager.warn_flag;
end

%% ===================== 6. 可视化结果 =====================
figure('Position', [100, 100, 1200, 1200]);

% 子图1：速度指令
subplot(7,1,1);
plot(T, Vx_ref, 'r', T, Vy_ref, 'g', T, Vturn_ref, 'b', 'LineWidth', 1.2);
title('底盘速度指令 (Vx/Vy/Vturn)');
ylabel('速度 (m/s)');
legend('Vx', 'Vy', 'Vturn');
grid on;

% 子图2：LQR转速跟踪
subplot(7,1,2);
plot(T, w_ref_log(1,:), 'r--', T, w_actual_log(1,:), 'r', 'LineWidth', 1); hold on;
plot(T, w_ref_log(2,:), 'g--', T, w_actual_log(2,:), 'g', 'LineWidth', 1);
plot(T, w_ref_log(3,:), 'b--', T, w_actual_log(3,:), 'b', 'LineWidth', 1);
plot(T, w_ref_log(4,:), 'k--', T, w_actual_log(4,:), 'k', 'LineWidth', 1);
yline(70*pi, 'm--', '电机最大转速', 'LineWidth', 1);
yline(-70*pi, 'm--', 'LineWidth', 1);
title('LQR转速跟踪（虚线=期望，实线=实际）');
ylabel('转速 (rad/s)');
grid on;

% 子图3：电机电流
subplot(7,1,3);
plot(T, I_actual_log(1,:), 'r', T, I_actual_log(2,:), 'g', ...
     T, I_actual_log(3,:), 'b', T, I_actual_log(4,:), 'k', 'LineWidth', 1);
title('4电机实际电流');
ylabel('电流 (-16384~16384)');
grid on;

% 子图4：功率曲线（卡尔曼估计/预测）
subplot(7,1,4);
plot(T, P_meas_log, 'g', 'LineWidth', 1.2, 'DisplayName', '观测功率'); hold on;
plot(T, P_est_log, 'b', 'LineWidth', 1.2, 'DisplayName', '卡尔曼估计功率');
plot(T, P_pred_log, 'm-.', 'LineWidth', 1.2, 'DisplayName', '卡尔曼预测功率');
P_limit = chassis.rule_manager.P_LIMIT_BLOOD * (P_limit_type == 1) + ...
          chassis.rule_manager.P_LIMIT_POWER * (P_limit_type ~= 1);
yline(P_limit, 'k-', ['功率上限=', num2str(P_limit), 'W'], 'LineWidth', 1.5);
title('功率曲线（卡尔曼+LQR控制）');
ylabel('功率 (W)');
legend('Location', 'best');
grid on;

% 子图5：缓冲能量
subplot(7,1,5);
plot(T, Z_log, 'm', 'LineWidth', 1.5); hold on;
yline(chassis.rule_manager.Z_MAX * chassis.rule_manager.WARN_THRESHOLD, 'r--', ...
    ['预警阈值（', num2str(chassis.rule_manager.WARN_THRESHOLD*100), '%）'], 'LineWidth', 1.2);
title('缓冲能量变化');
ylabel('缓冲能量');
ylim([0, chassis.rule_manager.Z_MAX+5]);
grid on;

% 子图6：预警状态
subplot(7,1,6);
stairs(T, warn_flag_log, 'r', 'LineWidth', 2);
title('预警状态（1=预警，0=正常）');
ylabel('预警状态');
ylim([-0.1, 1.1]);
grid on;

% 子图7：速度缩放系数
subplot(7,1,7);
plot(T, speed_scale_log, 'b', 'LineWidth', 1.5);
title('速度缩放系数（分级降速）');
ylabel('缩放系数');
xlabel('时间 (s)');
ylim([0, chassis.rule_manager.SPEED_SCALE_MAX+0.1]);
grid on;