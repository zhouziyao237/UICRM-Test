function plotFiveBar(Model, x, y, alpha, beta, y_direction)
    % 在笛卡尔坐标系中绘制并联五连杆机构
    % 输入:
    %   Model - 机构参数结构体
    %   x, y - 末端执行器坐标
    %   alpha, beta - 主动关节角度（弧度）
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
    if nargin < 5
        y_direction = 0;
    end
    
    if isnan(alpha) || isnan(beta)
        figure('Name', '并联五连杆机构', 'Position', [100, 100, 800, 600]);
        hold on;
        grid on;
        axis equal;
        
        % 设置坐标轴范围
        xlim([-200, 300]);
        ylim([-50, 300]);
        
        % 绘制坐标轴
        xlabel('X 轴 (mm)');
        ylabel('Y 轴 (mm)');
        title('并联五连杆机构笛卡尔坐标系');
        
        % 绘制基座
        D = [Model.L5, 0];
        plot([0, D(1)], [0, 0], 'k-', 'LineWidth', 3, 'DisplayName', '基座');
        
        % 绘制末端点
        plot(x, y, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'B点(末端)');
        
        % 标注不可达信息
        text(x+20, y+20, sprintf('位置 (%.1f, %.1f) 不可达', x, y), ...
            'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold', 'BackgroundColor', 'w');
        
        % 图例
        legend('Location', 'Best');
        
        hold off;
        return;
    end
    
    % 计算各关节点坐标
    O = [0, 0];  % 左侧基座
    D = [Model.L5, 0];  % 右侧基座
    
    % A点：左侧主动关节末端
    A = [Model.L3 * cos(alpha), Model.L3 * sin(alpha)];
    
    % C点：右侧主动关节末端
    C = [D(1) + Model.L1 * cos(beta), D(2) + Model.L1 * sin(beta)];
    
    % B点：末端执行器
    B = [x, y];
    
    % 创建图形
    figure('Name', '并联五连杆机构', 'Position', [100, 100, 800, 600]);
    hold on;
    grid on;
    axis equal;
    
    % 设置坐标轴范围
    x_min = min([0, D(1), A(1), C(1), B(1)]) - 50;
    x_max = max([0, D(1), A(1), C(1), B(1)]) + 50;
    y_min = min([0, D(2), A(2), C(2), B(2)]) - 50;
    y_max = max([0, D(2), A(2), C(2), B(2)]) + 50;
    
    xlim([x_min, x_max]);
    ylim([y_min, y_max]);
    
    % 设置Y轴方向
    if y_direction == 1
        set(gca, 'YDir', 'reverse');  % Y轴向下
        title('并联五连杆机构笛卡尔坐标系 (Y轴向下)');
    else
        set(gca, 'YDir', 'normal');  % Y轴向上
        title('并联五连杆机构笛卡尔坐标系 (Y轴向上)');
    end
    
    % 绘制坐标轴
    xlabel('X 轴 (mm)');
    ylabel('Y 轴 (mm)');
    
    % 绘制基座
    plot([0, D(1)], [0, 0], 'k-', 'LineWidth', 3, 'DisplayName', '基座');
    
    % 绘制连杆
    % L1: O -> A
    plot([O(1), A(1)], [O(2), A(2)], 'b-', 'LineWidth', 2, 'DisplayName', 'L1');
    
    % L2: A -> B
    plot([A(1), B(1)], [A(2), B(2)], 'b-', 'LineWidth', 2, 'DisplayName', 'L2');
    
    % L4: D -> C
    plot([D(1), C(1)], [D(2), C(2)], 'g-', 'LineWidth', 2, 'DisplayName', 'L4');
    
    % L3: C -> B
    plot([C(1), B(1)], [C(2), B(2)], 'g-', 'LineWidth', 2, 'DisplayName', 'L3');
    
    % 绘制关节点
    plot(O(1), O(2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'O点');
    plot(D(1), D(2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'D点');
    plot(A(1), A(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'A点');
    plot(C(1), C(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'C点');
    plot(B(1), B(2), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'B点(末端)');
    
    % 标注关节点名称
    text(O(1)-10, O(2)-10, 'O', 'FontSize', 12, 'FontWeight', 'bold');
    text(D(1)+10, D(2)-10, 'D', 'FontSize', 12, 'FontWeight', 'bold');
    text(A(1)-10, A(2)+10, 'A', 'FontSize', 12, 'FontWeight', 'bold');
    text(C(1)+10, C(2)+10, 'C', 'FontSize', 12, 'FontWeight', 'bold');
    text(B(1)+10, B(2)+10, 'B', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 标注角度
    text(O(1)+20, O(2)+20, sprintf('\\alpha = %.1f°', rad2deg(alpha)), ...
        'FontSize', 12, 'Color', 'b', 'FontWeight', 'bold');
    text(D(1)+20, D(2)+20, sprintf('\\beta = %.1f°', rad2deg(beta)), ...
        'FontSize', 12, 'Color', 'g', 'FontWeight', 'bold');
    
    % 标注末端坐标
    text(B(1)+20, B(2)-20, sprintf('(%.1f, %.1f)', x, y), ...
        'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
    
    % 绘制角度弧线
    % alpha角弧线
    if alpha > 0
        theta_alpha = linspace(0, alpha, 50);
        arc_alpha_x = O(1) + 30 * cos(theta_alpha);
        arc_alpha_y = O(2) + 30 * sin(theta_alpha);
        plot(arc_alpha_x, arc_alpha_y, 'b--', 'LineWidth', 1);
    end
    
    % beta角弧线
    if beta > 0
        theta_beta = linspace(0, beta, 50);
        arc_beta_x = D(1) + 30 * cos(theta_beta);
        arc_beta_y = D(2) + 30 * sin(theta_beta);
        plot(arc_beta_x, arc_beta_y, 'g--', 'LineWidth', 1);
    end
    
    % 添加连杆长度标注
    text((O(1)+A(1))/2, (O(2)+A(2))/2, sprintf('L3=%.0f', Model.L3), ...
        'FontSize', 10, 'Color', 'b', 'BackgroundColor', 'w');
    text((A(1)+B(1))/2, (A(2)+B(2))/2, sprintf('L4=%.0f', Model.L4), ...
        'FontSize', 10, 'Color', 'b', 'BackgroundColor', 'w');
    text((D(1)+C(1))/2, (D(2)+C(2))/2, sprintf('L1=%.0f', Model.L1), ...
        'FontSize', 10, 'Color', 'g', 'BackgroundColor', 'w');
    text((C(1)+B(1))/2, (C(2)+B(2))/2, sprintf('L2=%.0f', Model.L2), ...
        'FontSize', 10, 'Color', 'g', 'BackgroundColor', 'w');
    text((O(1)+D(1))/2, (O(2)+D(2))/2-20, sprintf('L5=%.0f', Model.L5), ...
        'FontSize', 10, 'Color', 'k', 'BackgroundColor', 'w');
    
    % 图例
    legend('Location', 'Best');
    
    hold off;
end