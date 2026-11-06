function MotionData = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts)
% MotionPlanning
% 实验四中已验证的梯形速度规划函数：根据位移和最大速度/加速度自动计算完成时间。
% 本课设主程序为了满足节拍，使用 SampleTimedLFPBTrajectory 按固定时间戳规划；
% 这里保留本函数，用于复现实验四或做单段自由时间规划。

N = length(TotalLength);
MotionData.ta = zeros(1, N);
MotionData.tv = zeros(1, N);
MotionData.td = zeros(1, N);
MotionData.vmax_real = zeros(1, N);
MotionData.acc_real = zeros(1, N);
MotionData.dec_real = zeros(1, N);
MotionData.dir = sign(TotalLength);
MotionData.Ts = Ts;

T = zeros(1, N);
ta = zeros(1, N); tv = zeros(1, N); td = zeros(1, N);

for i = 1:N
    L = abs(TotalLength(i));
    if L < 1e-6
        continue;
    end

    if L > (Vmax(i)^2)/(2*Accel(i)) + (Vmax(i)^2)/(2*Decel(i))
        ta(i) = Vmax(i)/Accel(i);
        td(i) = Vmax(i)/Decel(i);
        tv(i) = (L - 0.5*Accel(i)*ta(i)^2 - 0.5*Decel(i)*td(i)^2) / Vmax(i);
    else
        V_peak = sqrt(2*L*Accel(i)*Decel(i)/(Accel(i)+Decel(i)));
        ta(i) = V_peak/Accel(i);
        td(i) = V_peak/Decel(i);
        tv(i) = 0;
    end
    T(i) = ta(i) + tv(i) + td(i);
end

T_max = max(T);
if T_max == 0
    T_max = Ts;
end
MotionData.maxtime = T_max;

for i = 1:N
    if T(i) > 0
        k = T_max / T(i);
        MotionData.ta(i) = ta(i) * k;
        MotionData.tv(i) = tv(i) * k;
        MotionData.td(i) = td(i) * k;
        MotionData.vmax_real(i) = abs(TotalLength(i)) / ...
            (MotionData.ta(i)/2 + MotionData.tv(i) + MotionData.td(i)/2);
        MotionData.acc_real(i) = MotionData.vmax_real(i) / MotionData.ta(i);
        MotionData.dec_real(i) = MotionData.vmax_real(i) / MotionData.td(i);
    end
end
end
