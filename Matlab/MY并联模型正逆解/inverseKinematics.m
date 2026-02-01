
function result = inverseKinematics(Model, x, y)
    
    %   alpha _______ beta
    %        /   L5  \ 
    %    L3 /         \ L1
    %   θ_1 \         / θ_2
    %        \       /
    %    L4   \     /   L2
    %          \___/
    
    alpha_tmp = [0, 0];
    beta_tmp = [0, 0];
    t = x - Model.L5;

    K2 = (x^2 + y^2 + Model.L3^2 - Model.L4^2) / (2*Model.L3);
    K1 = (t^2 + y^2 + Model.L1^2 - Model.L2^2) / (2*Model.L1);

    % 限制 acos 参数范围，避免复数或 NaN
    arg_beta = K1 / sqrt(t^2 + y^2);
    arg_beta = max(min(arg_beta, 1), -1);
    
    arg_alpha = K2 / sqrt(x^2 + y^2);
    arg_alpha = max(min(arg_alpha, 1), -1);

    beta_tmp(1) = atan2(y, t) + acos(arg_beta);
    beta_tmp(2) = atan2(y, t) - acos(arg_beta);

    alpha_tmp(1) = atan2(y, x) + acos(arg_alpha);
    alpha_tmp(2) = atan2(y, x) - acos(arg_alpha);

    % 检查是否有 NaN 值
    if isnan(alpha_tmp(1)) || isnan(alpha_tmp(2)) || isnan(beta_tmp(1)) || isnan(beta_tmp(2))
        result.alpha = NaN;
        result.beta = NaN;
        return;
    end

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