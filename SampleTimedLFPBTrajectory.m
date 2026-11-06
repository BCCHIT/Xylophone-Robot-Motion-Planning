function [P, V, A, t] = SampleTimedLFPBTrajectory(waypoints, times, Ts, blendRatio)
% SampleTimedLFPBTrajectory
% 对带绝对时间戳的关键点进行逐段 LFPB 采样。
% 采样基于全局时间轴，不假设每一段时间都是 Ts 的整数倍。

if size(waypoints,1) ~= numel(times)
    error('waypoints 行数必须与 times 长度一致。');
end
if any(diff(times) < -1e-12)
    error('times 必须单调递增。');
end

tEnd = times(end);
t = (0:Ts:tEnd).';
if isempty(t) || abs(t(end) - tEnd) > 1e-12
    t = [t; tEnd];
end

N = numel(t);
P = zeros(N,3);
V = zeros(N,3);
A = zeros(N,3);

seg = 1;
nSeg = numel(times) - 1;

for k = 1:N
    tk = t(k);
    while seg < nSeg && tk > times(seg+1) + 1e-12
        seg = seg + 1;
    end

    t0 = times(seg);
    t1 = times(seg+1);
    T = t1 - t0;
    tau = min(max(tk - t0, 0), T);

    p0 = waypoints(seg,:);
    p1 = waypoints(seg+1,:);

    if T <= 1e-12
        P(k,:) = p1;
        V(k,:) = [0 0 0];
        A(k,:) = [0 0 0];
    else
        [s, sd, sdd] = LFPBScalar(tau, T, blendRatio);
        dp = p1 - p0;
        P(k,:) = p0 + s * dp;
        V(k,:) = sd * dp;
        A(k,:) = sdd * dp;
    end
end
end
