classdef SeasonRuleManager < handle
    % SeasonRuleManager: 赛季规则管理类（功率预警+缓冲计算）
    
    % 公共属性（外部可访问）
    properties (Access = public)
        Z;                % 当前缓冲能量
        warn_flag;        % 预警标志 false/true
        warn_type;        % 预警类型 0=无 1=缓冲 2=预测 3=立即
        speed_scale;      % 速度缩放系数
        SettlePeriod;     % 裁判结算周期
        P_LIMIT_BLOOD;    % 血量优先功率上限(W)
        P_LIMIT_POWER;    % 功率优先功率上限(W)
        Z_MAX;            % 缓冲上限
        WARN_THRESHOLD;   % 预警阈值（缓冲占比）
        SPEED_SCALE_MAX;  % 最大速度缩放系数
    end
    
    % 私有属性（仅内部访问）
    properties (Access = private)
        season_type;      % 赛季类型 '2025'/'2026'
    end
    
    methods
        % 构造函数
        function obj = SeasonRuleManager(season_type)
            obj.season_type = season_type;
            % 初始化赛季参数
            obj.init_season_params();
            
            obj.Z = obj.Z_MAX;       % 初始缓冲满格
            obj.warn_flag = false;
            obj.warn_type = 0;
            obj.speed_scale = 1.0;
        end
        
        % 初始化赛季参数
        function init_season_params(obj)
            switch obj.season_type
                case '2026'
                    obj.P_LIMIT_BLOOD = 75;
                    obj.P_LIMIT_POWER = 90;
                    obj.Z_MAX = 60;
                    obj.SettlePeriod = 0.1;
                    obj.WARN_THRESHOLD = 0.1;
                    obj.SPEED_SCALE_MAX = 1.2;
                case '2025'
                    obj.P_LIMIT_BLOOD = 60;
                    obj.P_LIMIT_POWER = 80;
                    obj.Z_MAX = 50;
                    obj.SettlePeriod = 0.2;
                    obj.WARN_THRESHOLD = 0.15;
                    obj.SPEED_SCALE_MAX = 1.1;
                otherwise
                    error('不支持的赛季类型！仅支持2025/2026');
            end
        end
        
        % 缓冲计算（仅镜像，不干预裁判）
        function calc_z_consume(obj, P_settle, P_limit, DT)
            if P_settle > P_limit
                obj.Z = obj.Z - (P_settle - P_limit) * DT;
            end
            % 2025赛季低功率恢复
            if strcmp(obj.season_type, '2025') && P_settle < P_limit
                obj.Z = obj.Z + (P_limit - P_settle) * DT * 0.5;
            end
            % 限幅0~Z_MAX
            obj.Z = max(min(obj.Z, obj.Z_MAX), 0);
        end
        
        % 预警判定+速度缩放
        function judge_warn(obj, P_meas, P_pred, P_limit_type)
            % 获取功率上限
            P_limit = obj.P_LIMIT_BLOOD * (P_limit_type == 1) + ...
                      obj.P_LIMIT_POWER * (P_limit_type ~= 1);
            
            % 重置预警状态
            obj.warn_flag = false;
            obj.warn_type = 0;
            
            % 1. 缓冲不足预警
            if obj.Z < obj.Z_MAX * obj.WARN_THRESHOLD
                obj.warn_flag = true;
                obj.warn_type = 1;
            end
            
            % 2. 预测功率超上限预警
            if P_pred > P_limit * 1.05
                obj.warn_flag = true;
                obj.warn_type = 2;
            end
            
            % 3. 实际功率超上限预警
            if P_meas > P_limit
                obj.warn_flag = true;
                obj.warn_type = 3;
            end
            
            % 分级速度缩放
            obj.calc_speed_scale(P_meas, P_limit);
        end
        
        % 计算速度缩放系数
        function calc_speed_scale(obj, P_meas, P_limit)
            if P_meas < 1e-6
                obj.speed_scale = 1.0;
                return;
            end
            
            switch obj.warn_type
                case 0  % 无预警
                    obj.speed_scale = min(obj.SPEED_SCALE_MAX, P_limit / P_meas);
                case 1  % 缓冲预警
                    obj.speed_scale = min(0.5, P_limit / P_meas);
                case 2  % 预测预警
                    obj.speed_scale = min(0.7, P_limit / P_meas);
                case 3  % 立即预警
                    obj.speed_scale = min(0.3, P_limit / P_meas);
            end
            % 最低保留0.1倍速度
            obj.speed_scale = max(obj.speed_scale, 0.1);
        end
        
        % 打印预警信息
        function print_warn_info(obj, t, P_meas, P_pred)
            if ~obj.warn_flag
                return;
            end
            warn_type_str = {
                '无预警', 
                '常规预警（缓冲不足）', 
                '预测预警（未来超功率）', 
                '立即预警（当前超功率）'
            };
            fprintf('【%s】时间=%.2fs | 实际功率=%.1fW | 预测功率=%.1fW | 剩余缓冲=%.1f\n', ...
                warn_type_str{obj.warn_type+1}, t, P_meas, P_pred, obj.Z);
        end
    end
end