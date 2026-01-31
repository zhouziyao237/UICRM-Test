function result = forwardKinematics(Model, alpha, beta)

%   alpha _______ beta
%        /   L5  \
%    L3 /         \ L1
%   θ_1 \         / θ_2
%        \       /
%    L4   \     /   L2
%          \___/

% 计算圆心坐标
C1 = [Model.L3*cos(alpha), Model.L3*sin(alpha)];   % 左侧圆心
C2 = [Model.L5 + Model.L1*cos(beta), Model.L1*sin(beta)]; % 右侧圆心
r1 = Model.L4;                               % 左侧半径
r2 = Model.L2;                               % 右侧半径

% 计算圆心距
dx = C2(1) - C1(1);
dy = C2(2) - C1(2);
d = sqrt(dx^2 + dy^2);

% 检查解的存在性
if d <= r1 + r2 && d >= abs(r1 - r2)
    % 计算交点
    a = (r1^2 - r2^2 + d^2) / (2*d);
    h = sqrt(r1^2 - a^2);
    
    % 中点坐标
    P2x = C1(1) + (a * dx) / d;
    P2y = C1(2) + (a * dy) / d;
    
    % 计算两个交点
    x1 = P2x + (h * (-dy)) / d;
    y1 = P2y + (h * dx) / d;
    
    x2 = P2x - (h * (-dy)) / d;
    y2 = P2y - (h * dx) / d;
    
    result.x1 = x1;
    result.x2 = x2;
    result.y1 = y1;
    result.y2 = y2;
else
    warning('两圆无交点: 圆心距 d = %.4f, 半径和 r1+r2 = %.4f, 半径差 |r1-r2| = %.4f', ...
        d, r1+r2, abs(r1-r2));
end
end