function FKPlotterDark(alpha, beta)
    % 并联五连杆正运动学求解与可视化 - 深色主题版
    % 输入: alpha, beta - 主动关节角度（弧度）
    %
    % 机构结构:
    %   alpht _______ beta
    %        /   L0  \ 
    %    L3 /         \ L1
    %       \         /
    %        \       /
    %    L4   \     /   L2
    %          \___/
    
    % 杆长参数（可调整）
    L0 = 4.0;    % 基座距离
    L1 = 2.0;    % 右侧主动杆长
    L2 = 3.0;    % 右侧从动杆长
    L3 = 2.0;    % 左侧主动杆长
    L4 = 3.0;    % 左侧从动杆长
    
    % 深色主题颜色定义
    dark_bg = [0.1, 0.1, 0.15];       % 背景色
    dark_axis = [0.2, 0.2, 0.25];     % 坐标轴背景
    light_text = [0.9, 0.9, 0.95];    % 文本颜色
    light_grid = [0.4, 0.4, 0.5];     % 网格颜色
    
    % 杆件颜色（高对比度）
    color_left_active = [0.9, 0.3, 0.3];     % 左侧主动杆 - 红色
    color_right_active = [0.3, 0.6, 0.9];    % 右侧主动杆 - 蓝色
    color_left_passive = [0.9, 0.6, 0.6];    % 左侧从动杆 - 浅红
    color_right_passive = [0.6, 0.8, 0.9];   % 右侧从动杆 - 浅蓝
    color_end = [0.4, 0.9, 0.4];             % 末端点 - 绿色
    color_fixed = [0.8, 0.8, 0.2];           % 固定点 - 黄色
    
    % 1. 建立两个圆方程
    % 圆1: (x - L3*cos(alpha))^2 + (y - L3*sin(alpha))^2 = L4^2
    % 圆2: (x - L0 - L1*cos(beta))^2 + (y - L1*sin(beta))^2 = L2^2
    
    % 2. 计算圆心坐标
    O1 = [L3 * cos(alpha), L3 * sin(alpha)];      % 左侧主动杆末端
    O2 = [L0 + L1 * cos(beta), L1 * sin(beta)];   % 右侧主动杆末端
    
    r1 = L4;  % 左侧圆半径
    r2 = L2;  % 右侧圆半径
    
    % 3. 计算圆心距
    d = norm(O2 - O1);
    
    % 4. 检查是否有解
    if d > r1 + r2 || d < abs(r1 - r2)
        fprintf('错误：两圆无交点，机构不可达！\n');
        fprintf('圆心距 d = %.4f, 半径和 = %.4f, 半径差 = %.4f\n', ...
                d, r1+r2, abs(r1-r2));
        return;
    end
    
    % 5. 计算交点（两圆交点公式）
    % 使用几何法：两圆交点求解
    a = (r1^2 - r2^2 + d^2) / (2*d);
    h = sqrt(r1^2 - a^2);
    
    % 中点P0
    P0 = O1 + a * (O2 - O1) / d;
    
    % 两个交点
    if abs(h) < 1e-10  % 相切情况
        solutions = P0;
        fprintf('注意：两圆相切，只有一个解\n');
    else
        % 垂直向量
        v = [-(O2(2) - O1(2)), O2(1) - O1(1)] * (h / d);
        
        % 两个交点
        solution1 = P0 + v;
        solution2 = P0 - v;
        solutions = [solution1; solution2];
    end
    
    % 6. 选择合理的解（通常选择y值较大的，即"上"构型）
    if size(solutions, 1) > 1
        % 这里选择y值较大的解（假设工作空间在上半平面）
        [~, idx] = max(solutions(:, 2));
        x = solutions(idx, 1);
        y = solutions(idx, 2);
        fprintf('找到2个解，选择解%d: (%.4f, %.4f)\n', idx, x, y);
    else
        x = solutions(1);
        y = solutions(2);
        fprintf('找到1个解: (%.4f, %.4f)\n', x, y);
    end
    
    % 7. 计算被动关节角度
    % 左侧被动角 θ1（L3和L4之间的夹角）
    dx1 = x - L3 * cos(alpha);
    dy1 = y - L3 * sin(alpha);
    theta1 = atan2(dy1, dx1) - alpha;
    
    % 右侧被动角 θ2（L1和L2之间的夹角）
    dx2 = x - L0 - L1 * cos(beta);
    dy2 = y - L1 * sin(beta);
    theta2 = atan2(dy2, dx2) - beta;
    
    % 8. 创建深色主题图形
    fig = figure('Position', [100, 100, 1200, 850]);
    set(fig, 'Color', dark_bg);
    
    % 设置所有子图的背景色
    set(0, 'DefaultAxesColor', dark_axis);
    set(0, 'DefaultAxesXColor', light_text);
    set(0, 'DefaultAxesYColor', light_text);
    set(0, 'DefaultTextColor', light_text);
    
    % 子图1：机构示意图
    subplot(2, 3, [1, 4]);
    hold on;
    grid on;
    box on;
    axis equal;
    
    % 设置当前子图颜色
    ax = gca;
    ax.Color = dark_axis;
    ax.GridColor = light_grid;
    ax.GridAlpha = 0.3;
    ax.XColor = light_text;
    ax.YColor = light_text;
    
    % 绘制固定点（带发光效果）
    plot(0, 0, 'o', 'MarkerSize', 14, 'MarkerFaceColor', color_fixed, ...
         'MarkerEdgeColor', 'w', 'LineWidth', 2);
    plot(L0, 0, 'o', 'MarkerSize', 14, 'MarkerFaceColor', color_fixed, ...
         'MarkerEdgeColor', 'w', 'LineWidth', 2);
    
    % 绘制主动杆（粗线）
    plot([0, L3*cos(alpha)], [0, L3*sin(alpha)], '-', ...
         'Color', color_left_active, 'LineWidth', 4);
    plot([L0, L0+L1*cos(beta)], [0, L1*sin(beta)], '-', ...
         'Color', color_right_active, 'LineWidth', 4);
    
    % 绘制从动杆（虚线）
    plot([L3*cos(alpha), x], [L3*sin(alpha), y], '--', ...
         'Color', color_left_passive, 'LineWidth', 3);
    plot([L0+L1*cos(beta), x], [L1*sin(beta), y], '--', ...
         'Color', color_right_passive, 'LineWidth', 3);
    
    % 绘制关节点（带发光效果）
    plot(L3*cos(alpha), L3*sin(alpha), 'o', 'MarkerSize', 10, ...
         'MarkerFaceColor', color_left_active, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    plot(L0+L1*cos(beta), L1*sin(beta), 'o', 'MarkerSize', 10, ...
         'MarkerFaceColor', color_right_active, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    
    % 绘制末端点（大而亮的点）
    plot(x, y, 'o', 'MarkerSize', 16, 'MarkerFaceColor', color_end, ...
         'MarkerEdgeColor', 'w', 'LineWidth', 2);
    
    % 标签（带阴影效果）
    text(-0.3, 0, 'A(0,0)', 'FontSize', 11, 'Color', light_text, ...
         'FontWeight', 'bold', 'BackgroundColor', [0.1, 0.1, 0.1, 0.7]);
    text(L0-0.3, 0, sprintf('E(%.1f,0)', L0), 'FontSize', 11, 'Color', light_text, ...
         'FontWeight', 'bold', 'BackgroundColor', [0.1, 0.1, 0.1, 0.7]);
    
    % 杆长标注
    text(L3*cos(alpha)/2-0.3, L3*sin(alpha)/2, sprintf('L3=%.1f', L3), ...
         'FontSize', 10, 'Color', color_left_active, 'FontWeight', 'bold');
    text((L0+L1*cos(beta)+L0)/2, L1*sin(beta)/2+0.2, sprintf('L1=%.1f', L1), ...
         'FontSize', 10, 'Color', color_right_active, 'FontWeight', 'bold');
    text((L3*cos(alpha)+x)/2-0.3, (L3*sin(alpha)+y)/2, sprintf('L4=%.1f', L4), ...
         'FontSize', 10, 'Color', color_left_passive, 'FontWeight', 'bold');
    text((L0+L1*cos(beta)+x)/2+0.3, (L1*sin(beta)+y)/2, sprintf('L2=%.1f', L2), ...
         'FontSize', 10, 'Color', color_right_passive, 'FontWeight', 'bold');
    
    % 末端点坐标标注
    text(x, y+0.4, sprintf('P(%.2f, %.2f)', x, y), ...
         'FontSize', 12, 'Color', color_end, 'FontWeight', 'bold', ...
         'BackgroundColor', [0.1, 0.1, 0.1, 0.7]);
    
    % 角度标注
    text(0.8, 0.3, sprintf('α=%.1f°', rad2deg(alpha)), ...
         'FontSize', 11, 'Color', color_left_active, 'FontWeight', 'bold');
    text(L0-0.8, 0.3, sprintf('β=%.1f°', rad2deg(beta)), ...
         'FontSize', 11, 'Color', color_right_active, 'FontWeight', 'bold');
    text(L3*cos(alpha)-0.6, L3*sin(alpha)+0.4, sprintf('θ1=%.1f°', rad2deg(theta1)), ...
         'FontSize', 10, 'Color', color_left_passive, 'FontWeight', 'bold');
    text(L0+L1*cos(beta)+0.4, L1*sin(beta)-0.4, sprintf('θ2=%.1f°', rad2deg(theta2)), ...
         'FontSize', 10, 'Color', color_right_passive, 'FontWeight', 'bold');
    
    title('并联五连杆机构示意图', 'FontSize', 14, 'FontWeight', 'bold', 'Color', light_text);
    xlabel('X坐标', 'FontSize', 12, 'FontWeight', 'bold', 'Color', light_text);
    ylabel('Y坐标', 'FontSize', 12, 'FontWeight', 'bold', 'Color', light_text);
    xlim([-1, L0+3]);
    ylim([-1, max([L3, L1, y])+1]);
    
    % 添加图例
    legend_items = {
        '左侧主动杆', '右侧主动杆', ...
        '左侧从动杆', '右侧从动杆', ...
        '固定点', '活动关节', '末端点'
    };
    legend(legend_items, 'Location', 'southoutside', 'NumColumns', 3, ...
           'TextColor', light_text, 'Color', [0.2, 0.2, 0.25]);
    
    % 子图2：圆方程求解示意图
    subplot(2, 3, 2);
    hold on;
    grid on;
    box on;
    axis equal;
    
    ax = gca;
    ax.Color = dark_axis;
    ax.GridColor = light_grid;
    ax.GridAlpha = 0.3;
    ax.XColor = light_text;
    ax.YColor = light_text;
    
    % 绘制两个圆
    theta_circle = linspace(0, 2*pi, 200);
    circle1_x = O1(1) + r1 * cos(theta_circle);
    circle1_y = O1(2) + r1 * sin(theta_circle);
    circle2_x = O2(1) + r2 * cos(theta_circle);
    circle2_y = O2(2) + r2 * sin(theta_circle);
    
    % 使用渐变色填充圆（半透明）
    patch(circle1_x, circle1_y, color_left_passive, 'FaceAlpha', 0.15, ...
          'EdgeColor', color_left_passive, 'LineWidth', 1.5);
    patch(circle2_x, circle2_y, color_right_passive, 'FaceAlpha', 0.15, ...
          'EdgeColor', color_right_passive, 'LineWidth', 1.5);
    
    % 绘制圆心
    plot(O1(1), O1(2), 'o', 'MarkerSize', 10, ...
         'MarkerFaceColor', color_left_active, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    plot(O2(1), O2(2), 'o', 'MarkerSize', 10, ...
         'MarkerFaceColor', color_right_active, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    
    % 绘制所有可能的解
    for i = 1:size(solutions, 1)
        if i == idx
            % 选中的解
            plot(solutions(i,1), solutions(i,2), 'o', 'MarkerSize', 14, ...
                 'MarkerFaceColor', color_end, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
            % 添加光环效果
            plot(solutions(i,1), solutions(i,2), 'o', 'MarkerSize', 20, ...
                 'MarkerFaceColor', 'none', 'MarkerEdgeColor', color_end, ...
                 'LineWidth', 1, 'LineStyle', '--');
        else
            % 另一个解
            plot(solutions(i,1), solutions(i,2), 'o', 'MarkerSize', 10, ...
                 'MarkerFaceColor', [0.8, 0.4, 0.8], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
        end
    end
    
    % 绘制圆心连线
    plot([O1(1), O2(1)], [O1(2), O2(2)], 'k--', 'LineWidth', 1);
    
    title('圆方程求解示意图', 'FontSize', 12, 'FontWeight', 'bold', 'Color', light_text);
    xlabel('X坐标', 'FontSize', 11, 'FontWeight', 'bold', 'Color', light_text);
    ylabel('Y坐标', 'FontSize', 11, 'FontWeight', 'bold', 'Color', light_text);
    
    % 添加标注
    text(O1(1), O1(2)-0.3, '圆心O1', 'FontSize', 10, 'Color', color_left_active, ...
         'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(O2(1), O2(2)-0.3, '圆心O2', 'FontSize', 10, 'Color', color_right_active, ...
         'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(mean([O1(1), O2(1)]), mean([O1(2), O2(2)]), sprintf('d=%.2f', d), ...
         'FontSize', 10, 'Color', light_text, 'FontWeight', 'bold', ...
         'BackgroundColor', [0.1, 0.1, 0.1, 0.7]);
    
    % 子图3：角度信息面板
    subplot(2, 3, 5);
    axis off;
    
    % 创建信息面板背景
    axes('Position', [0.67, 0.35, 0.28, 0.25]);
    fill([0, 1, 1, 0], [0, 0, 1, 1], [0.15, 0.15, 0.2], 'EdgeColor', light_grid, 'LineWidth', 1.5);
    axis off;
    
    % 显示文本信息
    axes('Position', [0.68, 0.36, 0.26, 0.23]);
    axis off;
    
    info_str = {
        '====== 机构参数 ======';
        sprintf('L0 = %.2f   (基座距离)', L0);
        sprintf('L1 = %.2f   (右侧主动杆)', L1);
        sprintf('L2 = %.2f   (右侧从动杆)', L2);
        sprintf('L3 = %.2f   (左侧主动杆)', L3);
        sprintf('L4 = %.2f   (左侧从动杆)', L4);
        ' ';
        '====== 输入角度 ======';
        sprintf('α  = %6.2f rad', alpha);
        sprintf('    = %6.1f°', rad2deg(alpha));
        sprintf('β  = %6.2f rad', beta);
        sprintf('    = %6.1f°', rad2deg(beta));
        ' ';
        '====== 正解结果 ======';
        sprintf('末端点 P: (%.4f, %.4f)', x, y);
        sprintf('θ1 = %6.2f° (被动角)', rad2deg(theta1));
        sprintf('θ2 = %6.2f° (被动角)', rad2deg(theta2));
        ' ';
        '====== 验证 ======';
        sprintf('圆1误差: %8.6f', abs((x-O1(1))^2 + (y-O1(2))^2 - L4^2));
        sprintf('圆2误差: %8.6f', abs((x-O2(1))^2 + (y-O2(2))^2 - L2^2));
    };
    
    text(0.05, 0.95, info_str, 'FontSize', 10, 'Color', light_text, ...
         'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
         'FontName', 'Consolas');
    
    title('计算结果', 'FontSize', 12, 'FontWeight', 'bold', 'Color', light_text);
    
    % 子图4：工作空间预览
    subplot(2, 3, [3, 6]);
    hold on;
    grid on;
    box on;
    
    ax = gca;
    ax.Color = dark_axis;
    ax.GridColor = light_grid;
    ax.GridAlpha = 0.3;
    ax.XColor = light_text;
    ax.YColor = light_text;
    
    % 采样工作空间点
    n_samples = 30;
    alpha_samples = linspace(0, pi, n_samples);
    beta_samples = linspace(0, pi, n_samples);
    
    workspace_points = [];
    
    for i = 1:n_samples
        for j = 1:n_samples
            % 计算当前角度对应的圆心
            O1_temp = [L3 * cos(alpha_samples(i)), L3 * sin(alpha_samples(i))];
            O2_temp = [L0 + L1 * cos(beta_samples(j)), L1 * sin(beta_samples(j))];
            
            d_temp = norm(O2_temp - O1_temp);
            
            % 检查是否可达
            if d_temp <= r1 + r2 && d_temp >= abs(r1 - r2)
                % 计算一个交点
                a_temp = (r1^2 - r2^2 + d_temp^2) / (2*d_temp);
                h_temp = sqrt(r1^2 - a_temp^2);
                P0_temp = O1_temp + a_temp * (O2_temp - O1_temp) / d_temp;
                v_temp = [-(O2_temp(2) - O1_temp(2)), O2_temp(1) - O1_temp(1)] * (h_temp / d_temp);
                P_temp = P0_temp + v_temp;
                
                workspace_points = [workspace_points; P_temp];
            end
        end
    end
    
    % 绘制工作空间点（使用散点图）
    if ~isempty(workspace_points)
        scatter(workspace_points(:,1), workspace_points(:,2), 8, ...
                'MarkerFaceColor', [0.4, 0.6, 0.9], 'MarkerEdgeColor', 'none', ...
                'MarkerFaceAlpha', 0.3);
    end
    
    % 标记当前计算的点
    plot(x, y, 'o', 'MarkerSize', 12, 'MarkerFaceColor', color_end, ...
         'MarkerEdgeColor', 'w', 'LineWidth', 2);
    text(x, y+0.2, '当前点', 'FontSize', 11, 'Color', color_end, ...
         'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % 绘制边界
    plot([0, L0, L0, 0, 0], [0, 0, max(ylim), max(ylim), 0], '--', ...
         'Color', light_grid, 'LineWidth', 1);
    
    title('工作空间预览', 'FontSize', 12, 'FontWeight', 'bold', 'Color', light_text);
    xlabel('X坐标', 'FontSize', 11, 'FontWeight', 'bold', 'Color', light_text);
    ylabel('Y坐标', 'FontSize', 11, 'FontWeight', 'bold', 'Color', light_text);
    xlim([-1, L0+3]);
    ylim([-1, max([L3, L1, y])+1]);
    
    % 9. 输出结果
    fprintf('\n=== 并联五连杆正运动学求解 ===\n');
    fprintf('输入角度: α = %.2f°, β = %.2f°\n', rad2deg(alpha), rad2deg(beta));
    fprintf('末端点坐标: (%.4f, %.4f)\n', x, y);
    fprintf('被动关节角: θ1 = %.2f°, θ2 = %.2f°\n', rad2deg(theta1), rad2deg(theta2));
    fprintf('圆方程验证误差: %.6f, %.6f\n\n', ...
            abs((x-O1(1))^2 + (y-O1(2))^2 - L4^2), ...
            abs((x-O2(1))^2 + (y-O2(2))^2 - L2^2));
    
    % 添加图形标题
    sgtitle(sprintf('并联五连杆正运动学 - 深色主题 (α=%.1f°, β=%.1f°)', ...
            rad2deg(alpha), rad2deg(beta)), 'FontSize', 16, ...
            'FontWeight', 'bold', 'Color', light_text);
end

% 辅助函数：弧度转角度
function deg = rad2deg(rad)
    deg = rad * 180 / pi;
end