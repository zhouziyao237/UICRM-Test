
function result = inverseKinematics(Model, x, y)

    alpha_tmp = [0, 0];
    beta_tmp = [0, 0];
    t = x - Model.L0;

    K2 = (x^2 + y^2 + Model.L3^2 - Model.L4^2) / (2*Model.L3);
    K1 = (t^2 + y^2 + Model.L1^2 - Model.L2^2) / (2*Model.L1);

    beta_tmp(0) = atan2(y, t) + arccos(K1 / sqrt(t^2 + y^2));
    beta_tmp(1) = atan2(y, t) - arccos(K1 / sqrt(t^2 + y^2));

    alpha_tmp(0) = atan2(y, x) + arccos(K2 / sqrt(x^2 + y^2));
    alpha_tmp(1) = atan2(y, x) - arccos(K2 / sqrt(x^2 + y^2));

    alpha_tmp(0) = mod(alpha_tmp(0), 2*pi);
    alpha_tmp(1) = mod(alpha_tmp(1), 2*pi);

    beta_tmp(0) = mod(beta_tmp(0), 2*pi);
    beta_tmp(1) = mod(beta_tmp(1), 2*pi);

    if alpha_tmp(1) >= pi/4
        alpha = alpha_tmp(1);
    else
        alpha = alpha_tmp(0);       
    end
    
    if beta_tmp(1) <= pi/4
        beta = beta_tmp(1);
    else
        beta = beta_tmp(0); 
    end

    result.alpha = alpha;
    result.beta = beta;

end