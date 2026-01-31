
% ------------------定义参数------------------------
Model.L1 = 150;
Model.L3 = 150;

Model.L2 = 250;
Model.L4 = 250;
Model.L5 = 108;

% 计算逆解
result = inverseKinematics(Model, 30, 150, 1);

% 显示结果
fprintf('逆解结果:\n');
fprintf('alpha = %.4f rad (%.2f°)\n', result.alpha, rad2deg(result.alpha));
fprintf('beta = %.4f rad (%.2f°)\n', result.beta, rad2deg(result.beta));

% 测试Y轴向下的绘制
plotFiveBar(Model, 30, 150, result.alpha, result.beta, 1);
                                        