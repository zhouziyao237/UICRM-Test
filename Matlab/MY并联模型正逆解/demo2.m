function IKAnimate()
    % 定义机构参数
    Model.L1 = 65;
    Model.L3 = 65;

    Model.L2 = 90;
    Model.L4 = 90;
    Model.L5 = 70;

    % 轨迹生成
    t = linspace(-10, 118, 128);
    offset = 80;  % 调整为更合理的Y方向偏移
    amp = 30;  % 振幅
    period = 128;  % 周期
    X_traj = t;
    Y_traj = offset + amp * sin((2*pi/period)*t);
    
    % Y轴方向控制
    y_direction = 1;  % Y轴向下
    
    % 创建图形
    figure('Name', '五连杆机构运动动画', 'Position', [100, 100, 800, 600]);
    hold on;
    grid on;
    axis equal;  % 保持画面比例不变
    
    % 设置坐标轴
    title('五连杆机构运动动画');
    xlabel('X 轴 (mm)');
    ylabel('Y 轴 (mm)');
    
    % 设置固定的坐标轴范围
    xlim([-50, 150]);  % X轴范围
    ylim([-50, 150]);  % Y轴范围
    
    % 设置Y轴方向
    if y_direction == 1
        set(gca, 'YDir', 'reverse');  % Y轴向下
    else
        set(gca, 'YDir', 'normal');  % Y轴向上
    end

    % 存储轨迹
    BTraj = [];
    
    % 初始化连杆
    hL1 = plot([0, 0], [0, 0], 'b-', 'LineWidth', 2, 'DisplayName', 'L3');
    hL2 = plot([0, 0], [0, 0], 'b-', 'LineWidth', 2, 'DisplayName', 'L4');
    hL3 = plot([0, 0], [0, 0], 'g-', 'LineWidth', 2, 'DisplayName', 'L2');
    hL4 = plot([0, 0], [0, 0], 'g-', 'LineWidth', 2, 'DisplayName', 'L1');
    hL5 = plot([0, 0], [0, 0], 'k-', 'LineWidth', 3, 'DisplayName', 'L5');

    % 初始化关节点
    hO = plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'O点');
    hD = plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'D点');
    hA = plot(0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'A点');
    hC = plot(0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'C点');
    hB = plot(0, 0, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'B点(末端)');
    
    % 初始化标注
    hAText = text(0, 0, 'A', 'FontSize', 12, 'FontWeight', 'bold');
    hBText = text(0, 0, 'B', 'FontSize', 12, 'FontWeight', 'bold');
    hCText = text(0, 0, 'C', 'FontSize', 12, 'FontWeight', 'bold');
    hDText = text(0, 0, 'D', 'FontSize', 12, 'FontWeight', 'bold');
    hOText = text(0, 0, 'O', 'FontSize', 12, 'FontWeight', 'bold');
    hAlphaBetaText = text(-150, 220, '\alpha:0° \newline\beta:0°', 'FontSize', 18, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
    hIKText = text(-150, 200, '逆解得到的角度', 'FontSize', 10, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
    
    % 初始化轨迹点
    hTrace = plot(NaN, NaN, 'r.', 'MarkerSize', 5, 'DisplayName', '轨迹');

    % 初始化角度弧线
    hArcAlpha = plot(NaN, NaN, 'b--', 'LineWidth', 1);
    hArcBeta = plot(NaN, NaN, 'g--', 'LineWidth', 1);

    % 初始化连杆长度标注
    hL3Text = text(0, 0, '', 'FontSize', 10, 'Color', 'b', 'BackgroundColor', 'w');
    hL4Text = text(0, 0, '', 'FontSize', 10, 'Color', 'b', 'BackgroundColor', 'w');
    hL1Text = text(0, 0, '', 'FontSize', 10, 'Color', 'g', 'BackgroundColor', 'w');
    hL2Text = text(0, 0, '', 'FontSize', 10, 'Color', 'g', 'BackgroundColor', 'w');
    hL5Text = text(0, 0, '', 'FontSize', 10, 'Color', 'k', 'BackgroundColor', 'w');

    % 初始化中间中点到足端的连线和标注
    hMidpointLine = plot(NaN, NaN, 'm--', 'LineWidth', 2, 'DisplayName', '中点到足端');
    hMidpoint = plot(NaN, NaN, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', '中点');
    hMidpointText = text(0, 0, 'M', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'm');
    hMidpointDistText = text(0, 0, '', 'FontSize', 10, 'Color', 'm', 'BackgroundColor', 'w');
    hMidpointAngleText = text(0, 0, '', 'FontSize', 10, 'Color', 'm', 'BackgroundColor', 'w');
    hMidpointArc = plot(NaN, NaN, 'm--', 'LineWidth', 1);

    % 开始动画
    while true
        for i = 1:length(X_traj)
            X = X_traj(i);
            Y = Y_traj(i);
            % 调用逆解函数
            IKResult = inverseKinematics(Model, X, Y);
            selected.alpha = IKResult.alpha;
            selected.beta = IKResult.beta;

            % 计算关键点坐标
            O = [0, 0];                             % 起点 O
            A = [Model.L3 * cos(selected.alpha), Model.L3 * sin(selected.alpha)];   % 左侧主动关节末端
            B = [X, Y];                            % 末端点
            D = [Model.L5, 0];                     % 右侧基座
            C = [D(1) + Model.L1 * cos(selected.beta), D(2) + Model.L1 * sin(selected.beta)]; % 右侧主动关节末端

            % 计算中间中点
            M = [(O(1)+D(1))/2, (O(2)+D(2))/2];  % 中间中点
            
            % 计算中点到足端的距离和角度
            midpointDistance = sqrt((B(1)-M(1))^2 + (B(2)-M(2))^2);
            midpointAngle = atan2(B(2)-M(2), B(1)-M(1));
            if midpointAngle < 0, midpointAngle = midpointAngle + 2*pi; end

            % 保存轨迹
            BTraj = [BTraj; B];

            % 更新连杆
            set(hL1, 'XData', [O(1), A(1)], 'YData', [O(2), A(2)]);
            set(hL2, 'XData', [A(1), B(1)], 'YData', [A(2), B(2)]);
            set(hL3, 'XData', [C(1), B(1)], 'YData', [C(2), B(2)]);
            set(hL4, 'XData', [D(1), C(1)], 'YData', [D(2), C(2)]);
            set(hL5, 'XData', [O(1), D(1)], 'YData', [O(2), D(2)]);

            % 更新中间中点到足端的连线
            set(hMidpointLine, 'XData', [M(1), B(1)], 'YData', [M(2), B(2)]);
            set(hMidpoint, 'XData', M(1), 'YData', M(2));

            % 更新关节点
            set(hO, 'XData', O(1), 'YData', O(2));
            set(hD, 'XData', D(1), 'YData', D(2));
            set(hA, 'XData', A(1), 'YData', A(2));
            set(hC, 'XData', C(1), 'YData', C(2));
            set(hB, 'XData', B(1), 'YData', B(2));

            % 更新标注位置
            set(hAText, 'Position', [A(1)-10, A(2)+10]);
            set(hBText, 'Position', [B(1)+10, B(2)+10]);
            set(hCText, 'Position', [C(1)+10, C(2)+10]);
            set(hDText, 'Position', [D(1)+10, D(2)-10]);
            set(hOText, 'Position', [O(1)-10, O(2)-10]);
            set(hMidpointText, 'Position', [M(1)-10, M(2)-10]);

            % 更新角度标注
            set(hAlphaBetaText, 'String', sprintf('\alpha:%.1f° \newline\beta:%.1f° \newline\rho:%.1f°', rad2deg(selected.alpha), rad2deg(selected.beta), rad2deg(midpointAngle)));

            % 更新轨迹
            set(hTrace, 'XData', BTraj(:,1), 'YData', BTraj(:,2));

            % 更新角度弧线
            if selected.alpha > 0
                theta_alpha = linspace(0, selected.alpha, 50);
                arc_alpha_x = O(1) + 30 * cos(theta_alpha);
                arc_alpha_y = O(2) + 30 * sin(theta_alpha);
                set(hArcAlpha, 'XData', arc_alpha_x, 'YData', arc_alpha_y);
            else
                set(hArcAlpha, 'XData', [], 'YData', []);
            end

            if selected.beta > 0
                theta_beta = linspace(0, selected.beta, 50);
                arc_beta_x = D(1) + 30 * cos(theta_beta);
                arc_beta_y = D(2) + 30 * sin(theta_beta);
                set(hArcBeta, 'XData', arc_beta_x, 'YData', arc_beta_y);
            else
                set(hArcBeta, 'XData', [], 'YData', []);
            end

            % 更新中点到足端的角度弧线
            if midpointAngle > 0
                theta_midpoint = linspace(0, midpointAngle, 50);
                arc_midpoint_x = M(1) + 40 * cos(theta_midpoint);
                arc_midpoint_y = M(2) + 40 * sin(theta_midpoint);
                set(hMidpointArc, 'XData', arc_midpoint_x, 'YData', arc_midpoint_y);
            else
                set(hMidpointArc, 'XData', [], 'YData', []);
            end

            % 更新连杆长度标注
            set(hL3Text, 'Position', [(O(1)+A(1))/2, (O(2)+A(2))/2], 'String', sprintf('L3=%.0f', Model.L3));
            set(hL4Text, 'Position', [(A(1)+B(1))/2, (A(2)+B(2))/2], 'String', sprintf('L4=%.0f', Model.L4));
            set(hL1Text, 'Position', [(D(1)+C(1))/2, (D(2)+C(2))/2], 'String', sprintf('L1=%.0f', Model.L1));
            set(hL2Text, 'Position', [(C(1)+B(1))/2, (C(2)+B(2))/2], 'String', sprintf('L2=%.0f', Model.L2));
            set(hL5Text, 'Position', [(O(1)+D(1))/2, (O(2)+D(2))/2-20], 'String', sprintf('L5=%.0f', Model.L5));
            
            % 更新中点到足端的距离和角度标注
            set(hMidpointDistText, 'Position', [(M(1)+B(1))/2, (M(2)+B(2))/2], 'String', sprintf('距离=%.1f', midpointDistance));
            set(hMidpointAngleText, 'Position', [M(1)+20, M(2)+20], 'String', sprintf('角度=%.1f°', rad2deg(midpointAngle)));

            % 保持固定的坐标轴范围，画面不移动
            % 注释掉动态调整坐标轴范围的代码

            % 刷新图形
            drawnow;
            pause(0.03);
        end
    end
end
