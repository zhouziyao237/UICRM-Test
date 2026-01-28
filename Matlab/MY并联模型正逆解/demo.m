% 示例1：测试正运动学
alpha = pi/3;  % 60度
beta = pi/4;   % 45度
FKPlotter(alpha, beta);

% 示例2：批量测试不同角度
% figure;
% hold on;
% for alpha = linspace(pi/6, pi/2, 5)
%     for beta = linspace(pi/3, 2*pi/3, 5)
%         % 计算正解（简化版）
%         L0=4; L1=2; L2=3; L3=2; L4=3;
%         O1 = [L3*cos(alpha), L3*sin(alpha)];
%         O2 = [L0+L1*cos(beta), L1*sin(beta)];
%         r1=L4; r2=L2;
%         d = norm(O2-O1);
%         if d <= r1+r2 && d >= abs(r1-r2)
%             a = (r1^2 - r2^2 + d^2)/(2*d);
%             h = sqrt(r1^2 - a^2);
%             P0 = O1 + a*(O2-O1)/d;
%             v = [-(O2(2)-O1(2)), O2(1)-O1(1)]*(h/d);
%             P = P0 + v;  % 选择一个解
%             plot(P(1), P(2), 'bo');
%         end
%     end
% end
% title('工作空间采样点');
% xlabel('X'); ylabel('Y');
% grid on;