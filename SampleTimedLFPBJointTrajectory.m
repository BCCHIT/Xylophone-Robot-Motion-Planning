function [q, qdRef, qddRef, t] = SampleTimedLFPBJointTrajectory(qWaypoints, times, Ts, blendRatio)
% SampleTimedLFPBJointTrajectory
% 按绝对时间戳对关节空间关键点进行 LFPB 采样。

if size(qWaypoints,1) ~= numel(times)
    error('qWaypoints row count must match times length.');
end
if any(diff(times) < -1e-12)
    error('times must be monotonic.');
end

tEnd = times(end);
t = (0:Ts:tEnd).';
if isempty(t) || abs(t(end) - tEnd) > 1e-12
    t = [t; tEnd];
end

N = numel(t);
q = zeros(N, size(qWaypoints,2));
qdRef = zeros(size(q));
qddRef = zeros(size(q));

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

    q0 = qWaypoints(seg,:);
    q1 = qWaypoints(seg+1,:);

    if T <= 1e-12
        q(k,:) = q1;
    else
        [s, sd, sdd] = LFPBScalar(tau, T, blendRatio);
        dq = q1 - q0;
        q(k,:) = q0 + s * dq;
        qdRef(k,:) = sd * dq;
        qddRef(k,:) = sdd * dq;
    end
end
end
