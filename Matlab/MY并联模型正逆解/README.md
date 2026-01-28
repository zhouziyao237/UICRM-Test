# 解算原理

## 运动学母方程 🔨

``` matlab

    %   alpht _______ beta
    %        /   L0  \ 
    %    L3 /         \ L1
    %       \         /
    %        \       /
    %    L4   \     /   L2
    %          \___/

    % x = [L0 + L1 * cos(beta) + L2 * cos(θ_2)]
    % x = [0 + L3 cos(alpht) + L4 * cos(θ_1)]

    % y = [L1 * sin(beta) + L2 * sin(θ_2)]
    % y = [L3 * sin(alpht) + L4 * sin(θ_1)]

    % cos(θ_2) = [x - L0 - L1 * cos(beta) + L2] / L2
    % sin(θ_2) = [y - L1 * sin(beta)] / L2

    % cos(θ_1) = [x - L3 * cos(alpht)] / L4
    % sin(θ_1) = [y - L3 * sin(alpht)] / L4

    % cos(θ_2)^2 + sin(θ_2)^2 = 1^2
    % [x - L0 - L1 * cos(beta) + L2]^2 * [y - L1 * sin(beta)]^2 = L2^2

    % cos(θ_1)^2 + sin(θ_1)^2 = 1^2
    % [x - L3 * cos(alpht)]^2 * [y - L3 * sin(alpht)]^2 = L4^2

```