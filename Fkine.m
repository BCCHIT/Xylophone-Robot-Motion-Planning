function g_st = Fkine(Xi, theta, g0)
% Fkine
% 基于指数积 POE 的正运动学。
% 输入：
%   Xi    6xN 运动旋量矩阵，每列为 [v; w]
%   theta 1xN 或 Nx1 关节角，单位 rad
%   g0    4x4 零位末端位姿
% 输出：
%   g_st  4x4 末端位姿

g_st = eye(4);
N = length(theta);

for i = 1:N
    v = Xi(1:3, i);
    w = Xi(4:6, i);
    th = theta(i);

    if norm(w) < 1e-6
        exp_xi = [eye(3), v * th; 0 0 0 1];
    else
        w_hat = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
        R = eye(3) + w_hat * sin(th) + w_hat^2 * (1 - cos(th));
        p = (eye(3) - R) * cross(w, v) + w * (w.' * v) * th;
        exp_xi = [R, p; 0 0 0 1];
    end

    g_st = g_st * exp_xi;
end

g_st = g_st * g0;
end
