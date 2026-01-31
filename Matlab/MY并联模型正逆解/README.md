# 解算原理

## 运动学母方程 🔨

```matlab

    %   alpha _______ beta
    %        /   L5  \ 
    %    L3 /         \ L1
    %   θ_1 \         / θ_2
    %        \       /
    %    L4   \     /   L2
    %          \___/

    % 右侧分支
    % x = [L5 + L1 * cos(beta) + L2 * cos(θ_2)]
    % y = [L1 * sin(beta) + L2 * sin(θ_2)]

    % 左侧分支
    % x = [0 + L3 cos(alpha) + L4 * cos(θ_1)]
    % y = [L3 * sin(alpha) + L4 * sin(θ_1)]

    % 被动关节表达式
    % cos(θ_2) = [x - L5 - L1 * cos(beta)] / L2
    % sin(θ_2) = [y - L1 * sin(beta)] / L2

    % cos(θ_1) = [x - L3 * cos(alpha)] / L4
    % sin(θ_1) = [y - L3 * sin(alpha)] / L4

    % 运动学约束方程（由cos²θ + sin²θ = 1得到）
    % [x - L5 - L1 * cos(beta)]^2 + [y - L1 * sin(beta)]^2 = L2^2   % 方程[1]
    % [x - L3 * cos(alpha)]^2 + [y - L3 * sin(alpha)]^2 = L4^2      % 方程[2]

```

## 逆解运动学 🔧

```matlab

    % ==================== 方程[1]化简：求解β ====================

    % 方程[1]: [x - L5 - L1 * cos(beta)]^2 + [y - L1 * sin(beta)]^2 = L2^2

    % 步骤1: 令 t = x - L5，简化方程
    t = x - L5

    % 步骤2: 展开方程
    % (t - L1 * cos(beta))^2 + (y - L1 * sin(beta))^2 = L2^2
    % t^2 - 2t*L1*cos(beta) + L1^2*cos^2(beta) + y^2 - 2y*L1*sin(beta) + L1^2*sin^2(beta) = L2^2

    % 步骤3: 利用cos²β + sin²β = 1
    % t^2 + y^2 + L1^2 - 2L1[t*cos(beta) + y*sin(beta)] = L2^2

    % 步骤4: 整理方程
    % 2L1[t*cos(beta) + y*sin(beta)] = t^2 + y^2 + L1^2 - L2^2

    % 步骤5: 标准化
    % t*cos(beta) + y*sin(beta) = (t^2 + y^2 + L1^2 - L2^2) / (2*L1)

    % 步骤6: 定义常数K1
    % K1 = (t^2 + y^2 + L1^2 - L2^2) / (2*L1)

    % 方程简化为：t*cos(beta) + y*sin(beta) = K1

    % 步骤7: 使用辅助角公式：a*cosθ + b*sinθ = R*cos(θ - φ)
    % 其中：R = √(a² + b²), φ = atan2(b, a)
    % 这里 a = t, b = y, 所以 R = √(t² + y²), φ = atan2(y, t)

    % 步骤8: 应用辅助角公式
    % √(t² + y²) * cos(beta - atan2(y, t)) = K1

    % 步骤9: 求解β
    % cos(beta - atan2(y, t)) = K1 / √(t² + y²)

    % 步骤10: 得到β的两个解
    % beta = atan2(y, t) ± arccos(K1 / √(t² + y²))

    % ==================== 方程[2]化简：求解α ====================

    % 方程[2]: [x - L3 * cos(alpha)]^2 + [y - L3 * sin(alpha)]^2 = L4^2

    % 步骤1: 展开方程
    % (x - L3*cos(alpha))^2 + (y - L3*sin(alpha))^2 = L4^2
    % x^2 - 2x*L3*cos(alpha) + L3^2*cos^2(alpha) + y^2 - 2y*L3*sin(alpha) + L3^2*sin^2(alpha) = L4^2

    % 步骤2: 利用cos²α + sin²α = 1
    % x^2 + y^2 + L3^2 - 2L3[x*cos(alpha) + y*sin(alpha)] = L4^2

    % 步骤3: 整理方程
    % 2L3[x*cos(alpha) + y*sin(alpha)] = x^2 + y^2 + L3^2 - L4^2

    % 步骤4: 标准化
    % x*cos(alpha) + y*sin(alpha) = (x^2 + y^2 + L3^2 - L4^2) / (2*L3)

    % 步骤5: 定义常数K2
    % K2 = (x^2 + y^2 + L3^2 - L4^2) / (2*L3)

    % 方程简化为：x*cos(alpha) + y*sin(alpha) = K2

    % 步骤6: 使用辅助角公式
    % 这里 a = x, b = y, 所以 R = √(x² + y²), φ = atan2(y, x)

    % 步骤7: 应用辅助角公式
    % √(x² + y²) * cos(alpha - atan2(y, x)) = K2

    % 步骤8: 求解α
    % cos(alpha - atan2(y, x)) = K2 / √(x² + y²)

    % 步骤9: 得到α的两个解
    % alpha = atan2(y, x) ± arccos(K2 / √(x² + y²))

    % ==================== 最终逆解公式 ====================

    % 输入：末端位置 (x, y)，连杆长度 L5, L1, L2, L3, L4
    % 输出：主动关节角度 α, β

    % 计算中间变量
    t = x - L5

    % 计算常数（注意：使用正号！）
    K1 = (t^2 + y^2 + L1^2 - L2^2) / (2 * L1)
    K2 = (x^2 + y^2 + L3^2 - L4^2) / (2 * L3)

    % 计算根号项
    R1 = sqrt(t^2 + y^2)  % 注意：是t²+y²，不是y²+x²
    R2 = sqrt(x^2 + y^2)

    % 计算基准角度
    phi_beta = atan2(y, t)    % β的基准角度
    phi_alpha = atan2(y, x)   % α的基准角度

    % 检查解的存在性
    if abs(K1 / R1) ≤ 1 且 abs(K2 / R2) ≤ 1
        % 计算角度差值
        delta_beta = arccos(K1 / R1)
        delta_alpha = arccos(K2 / R2)
        
        % 两个可能的解（±符号）
        beta1 = phi_beta + delta_beta
        beta2 = phi_beta - delta_beta
        
        alpha1 = phi_alpha + delta_alpha
        alpha2 = phi_alpha - delta_alpha
    else
        % 给定位置超出工作空间
        输出错误信息
    end

```

## 正解运动学 🔍

```matlab

    % ==================== 正解运动学：几何法推导 ====================

    % 已知：主动关节角度 α, β，连杆长度 L5, L1, L2, L3, L4
    % 求解：末端执行器坐标 (x, y)

    % ==================== 步骤1: 确定两圆的几何参数 ====================

    % 左侧圆：由主动关节α控制
    % 圆心C1：由L3绕原点旋转α角
    % C1x = L3 * cos(alpha)
    % C1y = L3 * sin(alpha)
    
    % 右侧圆：由主动关节β控制
    % 圆心C2：由L1绕点(L5, 0)旋转β角
    % C2x = L5 + L1 * cos(beta)
    % C2y = L1 * sin(beta)
    
    % 两圆半径
    % r1 = L4  % 左侧圆半径
    % r2 = L2  % 右侧圆半径

    % ==================== 步骤2: 写出两圆的方程 ====================

    % 左侧圆方程
    % (x - C1x)^2 + (y - C1y)^2 = r1^2  % 方程[3]
    
    % 右侧圆方程
    % (x - C2x)^2 + (y - C2y)^2 = r2^2  % 方程[4]

    % ==================== 步骤3: 计算圆心距 ====================

    % 计算两圆心之间的距离
    % dx = C2x - C1x
    % dy = C2y - C1y
    % d = sqrt(dx^2 + dy^2)

    % ==================== 步骤4: 检查解的存在性 ====================

    % 两圆相交的条件：|r1 - r2| ≤ d ≤ r1 + r2
    % if d > r1 + r2 || d < abs(r1 - r2)
    %     % 两圆无交点，机构不可达
    %     输出错误信息
    % else
    %     % 两圆有两个交点，机构可达
    % end

    % ==================== 步骤5: 几何法求解交点 ====================

    % 步骤5.1: 定义辅助变量
    % 在连心线C1C2上取一点P，使得P到交点的连线垂直于C1C2
    
    % 步骤5.2: 根据勾股定理建立方程
    % r1^2 = a^2 + h^2  % 方程[5] （左侧圆）
    % r2^2 = (d - a)^2 + h^2  % 方程[6] （右侧圆）
    % 其中：a是C1到P的距离，h是P到交点的距离
    
    % 步骤5.3: 消去h^2求解a
    % 用方程[5]减方程[6]：
    % r1^2 - r2^2 = a^2 - (d - a)^2
    % r1^2 - r2^2 = a^2 - (d^2 - 2ad + a^2)
    % r1^2 - r2^2 = 2ad - d^2
    
    % 整理得：
    % 2ad = r1^2 - r2^2 + d^2
    % a = (r1^2 - r2^2 + d^2) / (2 * d)

    % 步骤5.4: 求解h
    % 将a代入方程[5]：
    % h^2 = r1^2 - a^2
    % h = sqrt(r1^2 - a^2)

    % ==================== 步骤6: 计算交点坐标 ====================

    % 步骤6.1: 计算垂足P的坐标
    % P在连心线C1C2上，距离C1为a
    % Px = C1x + (a * dx) / d
    % Py = C1y + (a * dy) / d
    
    % 步骤6.2: 计算交点坐标
    % 交点位于P点的垂直方向上，距离为h
    % 垂直方向的单位向量：(-dy/d, dx/d) 和 (dy/d, -dx/d)
    
    % 交点1（上方）：
    % x1 = Px + (h * (-dy)) / d
    % y1 = Py + (h * dx) / d
    
    % 交点2（下方）：
    % x2 = Px - (h * (-dy)) / d
    % y2 = Py - (h * dx) / d

    % ==================== 步骤7: 选择有效解 ====================

    % 通常选择位于机构工作空间内的解
    % 或根据机构当前构型选择最近的解

    % ==================== 几何原理验证 ====================

    % 几何意义：
    % - 两圆交点即为末端执行器的可能位置
    % - 每个主动关节角度对应两个可能的末端位置
    % - 实际应用中需要根据机构构型选择合适的解

    % 数学验证：
    % 对于计算得到的(x1, y1)和(x2, y2)，应满足：
    % (x - C1x)^2 + (y - C1y)^2 = r1^2  % 左侧圆方程
    % (x - C2x)^2 + (y - C2y)^2 = r2^2  % 右侧圆方程

```

### 正解原理

正运动学是已知主动关节角度α和β，计算末端执行器坐标(x, y)的过程。根据机构几何关系，末端执行器坐标是两个圆的交点：

- **左侧圆**：圆心为 `[L3*cos(alpha), L3*sin(alpha)]`，半径为 `L4`
- **右侧圆**：圆心为 `[L5 + L1*cos(beta), L1*sin(beta)]`，半径为 `L2`

### 正解推导（代码对应版）

1. **定义圆心和半径**
   ```matlab
   C1 = [L3*cos(alpha), L3*sin(alpha)];   % 左侧圆心
   C2 = [L5 + L1*cos(beta), L1*sin(beta)]; % 右侧圆心
   r1 = L4;                               % 左侧半径
   r2 = L2;                               % 右侧半径
   ```

2. **计算圆心距**
   ```matlab
   dx = C2(1) - C1(1);
   dy = C2(2) - C1(2);
   d = sqrt(dx^2 + dy^2);
   ```

3. **检查解的存在性**
   ```matlab
   if d > r1 + r2 || d < abs(r1 - r2)
       % 两圆无交点，机构不可达
       输出错误信息
   end
   ```

4. **计算交点**
   ```matlab
   a = (r1^2 - r2^2 + d^2) / (2*d);
   h = sqrt(r1^2 - a^2);
   
   % 中点坐标
   P2x = C1(1) + (a * dx) / d;
   P2y = C1(2) + (a * dy) / d;
   
   % 两个可能的交点
   x1 = P2x + (h * (-dy)) / d;
   y1 = P2y + (h * dx) / d;
   
   x2 = P2x - (h * (-dy)) / d;
   y2 = P2y - (h * dx) / d;
   ```

5. **选择有效解**
   - 通常选择位于机构工作空间内的解
   - 或根据机构当前构型选择最近的解

## MATLAB实现代码

```matlab
function [alpha_solutions, beta_solutions] = inverse_kinematics_correct(x, y, L5, L1, L2, L3, L4)
    % 正确的逆运动学函数
    % 输入: x, y - 末端位置坐标
    %       L5, L1, L2, L3, L4 - 连杆长度
    % 输出: alpha_solutions - α角的可能解 (2×1数组)
    %       beta_solutions - β角的可能解 (2×1数组)
    
    % 计算中间变量 t
    t = x - L5;
    
    % 计算常数 K1, K2 (使用正号！)
    K1 = (t^2 + y^2 + L1^2 - L2^2) / (2 * L1);
    K2 = (x^2 + y^2 + L3^2 - L4^2) / (2 * L3);
    
    % 计算根号项
    R1 = sqrt(t^2 + y^2);   % 注意：是 t² + y²
    R2 = sqrt(x^2 + y^2);
    
    % 计算基准角度
    phi_beta = atan2(y, t);    % β的基准角度
    phi_alpha = atan2(y, x);   % α的基准角度
    
    % 初始化结果
    alpha_solutions = [];
    beta_solutions = [];
    
    % ==================== 求解β ====================
    if abs(K1 / R1) <= 1
        delta_beta = acos(K1 / R1);
        beta1 = phi_beta + delta_beta;
        beta2 = phi_beta - delta_beta;
        beta_solutions = [beta1, beta2];
    else
        warning('β分支不可达: |K1/R1| = %.4f > 1', abs(K1/R1));
    end
    
    % ==================== 求解α ====================
    if abs(K2 / R2) <= 1
        delta_alpha = acos(K2 / R2);
        alpha1 = phi_alpha + delta_alpha;
        alpha2 = phi_alpha - delta_alpha;
        alpha_solutions = [alpha1, alpha2];
    else
        warning('α分支不可达: |K2/R2| = %.4f > 1', abs(K2/R2));
    end
end
```

## 正解运动学 🔍

### 正解原理

正运动学是已知主动关节角度α和β，计算末端执行器坐标(x, y)的过程。根据机构几何关系，末端执行器坐标是两个圆的交点：

- **左侧圆**：圆心为 `[L3*cos(alpha), L3*sin(alpha)]`，半径为 `L4`
- **右侧圆**：圆心为 `[L5 + L1*cos(beta), L1*sin(beta)]`，半径为 `L2`

### 正解推导

1. **定义圆心和半径**
   ```matlab
   C1 = [L3*cos(alpha), L3*sin(alpha)];   % 左侧圆心
   C2 = [L5 + L1*cos(beta), L1*sin(beta)]; % 右侧圆心
   r1 = L4;                               % 左侧半径
   r2 = L2;                               % 右侧半径
   ```

2. **计算圆心距**
   ```matlab
   dx = C2(1) - C1(1);
   dy = C2(2) - C1(2);
   d = sqrt(dx^2 + dy^2);
   ```

3. **检查解的存在性**
   ```matlab
   if d > r1 + r2 || d < abs(r1 - r2)
       % 两圆无交点，机构不可达
       输出错误信息
   end
   ```

4. **计算交点**
   ```matlab
   a = (r1^2 - r2^2 + d^2) / (2*d);
   h = sqrt(r1^2 - a^2);
   
   % 中点坐标
   P2x = C1(1) + (a * dx) / d;
   P2y = C1(2) + (a * dy) / d;
   
   % 两个可能的交点
   x1 = P2x + (h * (-dy)) / d;
   y1 = P2y + (h * dx) / d;
   
   x2 = P2x - (h * (-dy)) / d;
   y2 = P2y - (h * dx) / d;
   ```

5. **选择有效解**
   - 通常选择位于机构工作空间内的解
   - 或根据机构当前构型选择最近的解

### MATLAB实现代码

```matlab
function [x_solutions, y_solutions] = forward_kinematics(alpha, beta, L5, L1, L2, L3, L4)
    % 正运动学函数
    % 输入: alpha, beta - 主动关节角度
    %       L5, L1, L2, L3, L4 - 连杆长度
    % 输出: x_solutions - x坐标的可能解 (1×2数组)
    %       y_solutions - y坐标的可能解 (1×2数组)
    
    % 计算圆心坐标
    C1 = [L3*cos(alpha), L3*sin(alpha)];   % 左侧圆心
    C2 = [L5 + L1*cos(beta), L1*sin(beta)]; % 右侧圆心
    r1 = L4;                               % 左侧半径
    r2 = L2;                               % 右侧半径
    
    % 计算圆心距
    dx = C2(1) - C1(1);
    dy = C2(2) - C1(2);
    d = sqrt(dx^2 + dy^2);
    
    % 初始化结果
    x_solutions = [];
    y_solutions = [];
    
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
        
        x_solutions = [x1, x2];
        y_solutions = [y1, y2];
    else
        warning('两圆无交点: 圆心距 d = %.4f, 半径和 r1+r2 = %.4f, 半径差 |r1-r2| = %.4f', ...
            d, r1+r2, abs(r1-r2));
    end
end
```

## 使用示例

```matlab
% 定义机构参数
L5 = 5;   % 水平基准距离
L1 = 3;   % 右侧第一连杆长度
L2 = 4;   % 右侧第二连杆长度
L3 = 3;   % 左侧第一连杆长度
L4 = 4;   % 左侧第二连杆长度

% 测试位置
x_test = 6;
y_test = 5;

% 计算逆解
[alpha_sol, beta_sol] = inverse_kinematics_correct(x_test, y_test, L5, L1, L2, L3, L4);

if ~isempty(alpha_sol) && ~isempty(beta_sol)
    fprintf('目标位置: (%.2f, %.2f)\n', x_test, y_test);
    fprintf('α角解: %.4f rad, %.4f rad\n', alpha_sol(1), alpha_sol(2));
    fprintf('β角解: %.4f rad, %.4f rad\n', beta_sol(1), beta_sol(2));
    
    % 验证解的正确性
    % 选择一组解(α1, β1)计算正解
    alpha = alpha_sol(1);
    beta = beta_sol(1);
    
    % 使用正运动学函数计算正解
    [x_sol, y_sol] = forward_kinematics(alpha, beta, L5, L1, L2, L3, L4);
    
    fprintf('正解验证: (%.6f, %.6f)\n', x_sol(1), y_sol(1));
    fprintf('位置误差: dx = %.6f, dy = %.6f\n', abs(x_sol(1) - x_test), abs(y_sol(1) - y_test));
end

% 正运动学测试示例
alpha_test = pi/4;  % 45度
beta_test = pi/3;   % 60度

[x_fwd, y_fwd] = forward_kinematics(alpha_test, beta_test, L5, L1, L2, L3, L4);

if ~isempty(x_fwd)
    fprintf('\n正运动学测试:\n');
    fprintf('主动关节角度: α = %.4f rad, β = %.4f rad\n', alpha_test, beta_test);
    fprintf('末端坐标解1: (%.4f, %.4f)\n', x_fwd(1), y_fwd(1));
    fprintf('末端坐标解2: (%.4f, %.4f)\n', x_fwd(2), y_fwd(2));
end
```
