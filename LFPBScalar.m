function [s, sd, sdd] = LFPBScalar(tau, T, blendRatio)
% LFPBScalar
% 标准 0->1 的线性函数加抛物线过渡（Linear Function with Parabolic Blends）。
% tau 为当前段内时间，T 为当前段总时长。

if T <= 0
    s = 1; sd = 0; sdd = 0;
    return;
end

ratio = min(max(blendRatio, 1e-6), 0.499);
tb = ratio * T;
V = 1 / (T - tb);
a = V / tb;

tau = min(max(tau, 0), T);

if tau < tb
    s = 0.5 * a * tau^2;
    sd = a * tau;
    sdd = a;
elseif tau <= T - tb
    s = V * (tau - tb/2);
    sd = V;
    sdd = 0;
else
    dt = T - tau;
    s = 1 - 0.5 * a * dt^2;
    sd = a * dt;
    sdd = -a;
end

% 数值保险
s = min(max(s, 0), 1);
end
