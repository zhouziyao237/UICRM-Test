
function result = inverseKinematics(Model, x, y, y_direction)
    % 五连杆机构逆运动学计算
    % 输入:
    %   Model - 机构参数结构体
    %   x, y - 末端执行器坐标
    %   y_direction - Y轴方向（可选）
    %                0 - Y轴向上（默认）
    %                1 - Y轴向下
    
    %   alpha _______ beta
    %        /   L5  \ 
    %    L3 /         \ L1
    %   θ_1 \         / θ_2
    %        \       /
    %    L4   \     /   L2
    %          \___/

    % 默认Y轴向上
    if nargin < 4
        y_direction = 0;
    end
    
    % 根据Y轴方向调整y坐标
    if y_direction == 1
        y_eff = -y;  % Y轴向下时，y坐标取反
    else
        y_eff = y;   % Y轴向上时，y坐标不变
    end
    
    alpha_tmp = [0, 0];
    beta_tmp = [0, 0];
    t = x - Model.L5;

    K2 = (x^2 + y^2 + Model.L3^2 - Model.L4^2) / (2*Model.L3);
    K1 = (t^2 + y^2 + Model.L1^2 - Model.L2^2) / (2*Model.L1);

    beta_tmp(1) = atan2(y, t) + acos(K1 / sqrt(t^2 + y^2));
    beta_tmp(2) = atan2(y, t) - acos(K1 / sqrt(t^2 + y^2));

    alpha_tmp(1) = atan2(y, x) + acos(K2 / sqrt(x^2 + y^2));
    alpha_tmp(2) = atan2(y, x) - acos(K2 / sqrt(x^2 + y^2));

    alpha_tmp(1) = mod(alpha_tmp(1), 2*pi);
    alpha_tmp(2) = mod(alpha_tmp(2), 2*pi);

    beta_tmp(1) = mod(beta_tmp(1), 2*pi);
    beta_tmp(2) = mod(beta_tmp(2), 2*pi);

    if alpha_tmp(1) >= pi/4
        alpha = alpha_tmp(1);
    else
        alpha = alpha_tmp(2);       
    end
    
    if beta_tmp(1) <= pi/4
        beta = beta_tmp(1);
    else
        beta = beta_tmp(2); 
    end

    result.alpha = alpha;
    result.beta = beta;

end