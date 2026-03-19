classdef KFPowerEstimator < handle
    % KFPowerEstimator: 卡尔曼滤波功率估计/预测类
    % 核心功能：估计当前功率+预测未来100ms功率
    
    properties (Access = private)
        A;          % 状态转移矩阵 [1, DT; 0, 1]
        B;          % 输入矩阵 [0; 0]
        H;          % 观测矩阵 [1, 0]
        Q;          % 过程噪声协方差
        R;          % 观测噪声协方差
        P;          % 状态协方差矩阵
        x_est;      % 状态估计 [功率; 功率变化率]
        DT;         % 控制周期(s)
    end
    
    properties (Access = public)
        P_est;      % 当前估计功率(W)
        P_pred;     % 预测未来100ms功率(W)
    end
    
    methods
        % 构造函数
        function obj = KFPowerEstimator(DT)
            obj.DT = DT;
            % 初始化KF参数
            obj.A = [1, DT; 0, 1];
            obj.B = [0; 0];
            obj.H = [1, 0];
            obj.Q = diag([0.1, 0.01]);  % 过程噪声
            obj.R = 1.0;                % 观测噪声
            obj.P = eye(2);             % 状态协方差
            obj.x_est = [0; 0];         % 初始状态
            
            obj.P_est = 0;
            obj.P_pred = 0;
        end
        
        % KF更新（输入观测功率，输出估计/预测功率）
        function update(obj, P_meas)
            % 1. 预测步
            x_pred = obj.A * obj.x_est + obj.B * 0;  % 无输入
            P_pred_mat = obj.A * obj.P * obj.A' + obj.Q;
            
            % 2. 更新步
            y = P_meas - obj.H * x_pred;             % 观测残差
            S = obj.H * P_pred_mat * obj.H' + obj.R; % 残差协方差
            K = P_pred_mat * obj.H' / S;             % 卡尔曼增益
            
            % 修正状态
            obj.x_est = x_pred + K * y;
            obj.P = (eye(2) - K * obj.H) * P_pred_mat;
            
            % 输出当前估计功率
            obj.P_est = obj.x_est(1);
            
            % 预测未来100ms功率
            A_100 = [1, 0.1; 0, 1];  % 100ms状态转移矩阵
            x_pred_future = A_100 * obj.x_est;
            obj.P_pred = x_pred_future(1);
        end
        
        % 重置KF状态
        function reset(obj)
            obj.x_est = [0; 0];
            obj.P = eye(2);
            obj.P_est = 0;
            obj.P_pred = 0;
        end
    end
end