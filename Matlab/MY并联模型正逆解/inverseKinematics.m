
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