function [qd, qdd] = NumericalDiff(t, q)
% NumericalDiff
% 支持非均匀时间轴的数值微分。输入 q 为 N x M。

N = size(q,1);
M = size(q,2);
qd = zeros(N,M);
qdd = zeros(N,M);

if N < 2
    return;
end

% 一阶差分。这里假设 q 已在主程序或选解阶段完成连续化，不能再用
% wrapToPi 处理差值，否则第 6 轴超过 180 deg 的真实连续运动会被误折返。
for i = 2:N
    dt = t(i) - t(i-1);
    if dt <= 0, dt = eps; end
    dq = q(i,:) - q(i-1,:);
    qd(i,:) = dq / dt;
end
qd(1,:) = qd(2,:);

% 二阶差分
for i = 2:N
    dt = t(i) - t(i-1);
    if dt <= 0, dt = eps; end
    qdd(i,:) = (qd(i,:) - qd(i-1,:)) / dt;
end
qdd(1,:) = qdd(2,:);
end
