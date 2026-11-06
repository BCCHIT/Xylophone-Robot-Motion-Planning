function [q, qd, qdd] = TrajGena(time, MotionData, loc1, loc2)
% TrajGena
% 实验四中已验证的插补点生成函数。与 MotionPlanning 配套使用。

N = length(loc1);
q = zeros(1,N);
qd = zeros(1,N);
qdd = zeros(1,N);

for i = 1:N
    dir = MotionData.dir(i);
    ta = MotionData.ta(i);
    tv = MotionData.tv(i);
    td = MotionData.td(i);
    v = MotionData.vmax_real(i);
    a = MotionData.acc_real(i);
    d = MotionData.dec_real(i);

    if time <= ta
        q(i) = loc1(i) + dir * 0.5 * a * time^2;
        qd(i) = dir * a * time;
        qdd(i) = dir * a;
    elseif time <= ta + tv
        q(i) = loc1(i) + dir * (0.5 * a * ta^2 + v * (time - ta));
        qd(i) = dir * v;
        qdd(i) = 0;
    elseif time <= ta + tv + td
        t_dec = time - ta - tv;
        q(i) = loc1(i) + dir * (0.5 * a * ta^2 + v * tv + v * t_dec - 0.5 * d * t_dec^2);
        qd(i) = dir * (v - d * t_dec);
        qdd(i) = -dir * d;
    else
        q(i) = loc2(i);
        qd(i) = 0;
        qdd(i) = 0;
    end
end
end
