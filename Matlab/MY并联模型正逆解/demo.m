
% ------------------定义参数------------------------
% 定义机构参数
Model.L1 = 130;
Model.L2 = 210;
Model.L3 = 130;
Model.L4 = 210;
Model.L5 = 90;

x = 0;
y =200;

% 计算逆解
result = inverseKinematics(Model, x, y);
result1 = forwardKinematics(Model, result.alpha, result.beta);

% 检查结果
fprintf('正解结果:\n');
fprintf('x = %.4f\n', result1.x1);
fprintf('y = %.4f\n', result1.y1);

% 显示结果
fprintf('逆解结果:\n');
fprintf('alpha = %.4f rad (%.2f°)\n', result.alpha, rad2deg(result.alpha));
fprintf('beta = %.4f rad (%.2f°)\n', result.beta, rad2deg(result.beta));

% 测试Y轴向下的绘制
plotFiveBar(Model, x, y, result.alpha, result.beta, 1);
                                        