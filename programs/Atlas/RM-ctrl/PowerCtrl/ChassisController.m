classdef ChassisController < handle
    % ChassisController: 底盘控制主类（LQR+卡尔曼）
    % 核心：麦轮逆解 + LQR速度跟踪 + 卡尔曼功率预测 + 功率预警
    
    properties (Access = private)
        % 底盘物理参数
        WHEEL_RADIUS;     % 轮子半径(m)
        CHASSIS_WIDTH;    % 底盘宽度(m)
        CHASSIS_LENGTH;   % 底盘长度(m)
        MOTOR_MAX_SPEED;  % 电机最大转速(rad/s)
        
        % LQR参数
        A_lqr;            % 状态矩阵
        B_lqr;            % 输入矩阵
        Q_lqr;            % 状态权重
        R_lqr;            % 输入权重
        K_lqr;            % LQR增益
        
        % 运行参数
        DT;               % 控制周期(s)
        P_limit_type;     % 功率模式 1=血量 2=功率
        P_settle_buf;     % 结算功率缓存
        settle_step;      % 裁判结算步数
    end
    
    % 关键修复：将需要外部访问的组件类改为公共属性
    properties (Access = public)
        % 组件类（外部可访问）
        kf_estimator;     % 卡尔曼功率估计器
        rule_manager;     % 赛季规则管理器
        
        % 输出状态
        w_ref;            % 电机期望转速(4x1)
        w_actual;         % 电机实际转速(4x1)
        dw_lqr;           % LQR转速修正量(4x1)
        I_actual;         % 电机实际电流(4x1)
        P_meas;           % 观测功率(W)
        speed_scale;      % 速度缩放系数
    end
    
    methods
        % 构造函数
        function obj = ChassisController(DT, season_type, P_limit_type)
            % 初始化底盘参数（RM 3508麦轮通用）
            obj.WHEEL_RADIUS = 0.06;
            obj.CHASSIS_WIDTH = 0.5;
            obj.CHASSIS_LENGTH = 0.5;
            obj.MOTOR_MAX_SPEED = 70*pi;  % 70π rad/s
            
            % 初始化运行参数
            obj.DT = DT;
            obj.P_limit_type = P_limit_type;
            
            % 初始化组件类（公共属性，外部可访问）
            obj.kf_estimator = KFPowerEstimator(DT);
            obj.rule_manager = SeasonRuleManager(season_type);
            
            % 初始化LQR
            obj.init_lqr();
            
            % 初始化输出状态
            obj.w_ref = zeros(4, 1);
            obj.w_actual = zeros(4, 1);
            obj.dw_lqr = zeros(4, 1);
            obj.I_actual = zeros(4, 1);
            obj.P_meas = 0;
            obj.speed_scale = 1.0;
            
            % 计算裁判结算步数
            obj.settle_step = obj.rule_manager.SettlePeriod / DT;
            obj.P_settle_buf = [];
        end
        
        % 初始化LQR参数
        function init_lqr(obj)
            % 电机动力学模型（简化4电机）
            b = 0.0005;  % 阻尼系数
            J = 0.001;   % 转动惯量
            obj.A_lqr = diag([-b/J, -b/J, -b/J, -b/J]);  % 4x4状态矩阵
            obj.B_lqr = eye(4);                           % 4x4输入矩阵
            
            % LQR权重（跟踪精度+输入平滑）
            obj.Q_lqr = diag([300, 300, 300, 300]);  % 转速跟踪权重
            obj.R_lqr = diag([1, 1, 1, 1]);          % 输入平滑权重
            
            % 计算LQR增益
            [obj.K_lqr, ~, ~] = lqr(obj.A_lqr, obj.B_lqr, obj.Q_lqr, obj.R_lqr);
        end
        
        % 底盘控制主逻辑（输入速度指令，输出电机转速）
        function control(obj, Vx, Vy, Vturn, t)
            % 1. 应用速度缩放（功率预警）
            Vx_limit = Vx * obj.rule_manager.speed_scale;
            Vy_limit = Vy * obj.rule_manager.speed_scale;
            Vturn_limit = Vturn * obj.rule_manager.speed_scale;
            
            % 2. 麦轮逆解（计算期望转速）
            obj.w_ref = obj.wheel_inverse_kinematics(Vx_limit, Vy_limit, Vturn_limit);
            
            % 3. LQR速度闭环（转速修正）
            w_error = obj.w_actual - obj.w_ref;
            obj.dw_lqr = -obj.K_lqr * w_error;
            obj.dw_lqr = obj.clamp(obj.dw_lqr, -80, 80);  % 修正量限幅
            
            % 4. 更新实际转速（限幅）
            obj.w_actual = obj.w_actual + obj.dw_lqr * obj.DT;
            obj.w_actual = obj.clamp(obj.w_actual, -obj.MOTOR_MAX_SPEED, obj.MOTOR_MAX_SPEED);
            
            % 5. 模拟电机电流+计算功率
            obj.calc_motor_current_power();
            
            % 6. 卡尔曼功率估计/预测
            obj.kf_estimator.update(obj.P_meas);
            
            % 7. 裁判周期功率结算
            obj.P_settle_buf = [obj.P_settle_buf, obj.P_meas];
            if length(obj.P_settle_buf) >= obj.settle_step
                P_settle = mean(obj.P_settle_buf);
                % 获取功率上限
                P_limit = obj.rule_manager.P_LIMIT_BLOOD * (obj.P_limit_type == 1) + ...
                          obj.rule_manager.P_LIMIT_POWER * (obj.P_limit_type ~= 1);
                % 缓冲计算
                obj.rule_manager.calc_z_consume(P_settle, P_limit, obj.DT);
                % 清空缓存
                obj.P_settle_buf = [];
            end
            
            % 8. 功率预警判定
            obj.rule_manager.judge_warn(obj.P_meas, obj.kf_estimator.P_pred, obj.P_limit_type);
            obj.speed_scale = obj.rule_manager.speed_scale;
            
            % 9. 打印预警信息
            obj.rule_manager.print_warn_info(t, obj.P_meas, obj.kf_estimator.P_pred);
        end
        
        % 麦轮逆解
        function w_ref = wheel_inverse_kinematics(obj, Vx, Vy, Vturn)
            w_ref = zeros(4, 1);
            L = obj.CHASSIS_LENGTH / 2;
            W = obj.CHASSIS_WIDTH / 2;
            w_ref(1) = (Vx - Vy - (L + W)*Vturn) / obj.WHEEL_RADIUS;
            w_ref(2) = (Vx + Vy + (L + W)*Vturn) / obj.WHEEL_RADIUS;
            w_ref(3) = (Vx + Vy - (L + W)*Vturn) / obj.WHEEL_RADIUS;
            w_ref(4) = (Vx - Vy + (L + W)*Vturn) / obj.WHEEL_RADIUS;
        end
        
        % 模拟电机电流+计算功率
        function calc_motor_current_power(obj)
            I_MAX = 16384;    % 3508电机电流上限
            U_BAT = 24;       % 电池电压(V)
            ALPHA = 0.2;      % 滤波系数
            
            % 模拟电机电流（基于转速）
            for i = 1:4
                obj.I_actual(i) = (obj.w_actual(i)/1000)*I_MAX + 800 * randn();
                obj.I_actual(i) = obj.clamp(obj.I_actual(i), -I_MAX, I_MAX);
            end
            
            % 计算功率（滤波）
            sum_I = sum(abs(obj.I_actual));
            P_raw = U_BAT * sum_I / I_MAX;
            obj.P_meas = ALPHA * P_raw + (1 - ALPHA) * obj.P_meas;
        end
        
        % 数值限幅
        function y = clamp(obj, x, min_val, max_val)
            y = max(min(x, max_val), min_val);
        end
        
        % 重置控制器
        function reset(obj)
            obj.w_ref = zeros(4, 1);
            obj.w_actual = zeros(4, 1);
            obj.dw_lqr = zeros(4, 1);
            obj.I_actual = zeros(4, 1);
            obj.P_meas = 0;
            obj.speed_scale = 1.0;
            obj.kf_estimator.reset();
            obj.rule_manager.Z = obj.rule_manager.Z_MAX;
            obj.rule_manager.warn_flag = false;
            obj.rule_manager.warn_type = 0;
            obj.P_settle_buf = [];
        end
    end
end