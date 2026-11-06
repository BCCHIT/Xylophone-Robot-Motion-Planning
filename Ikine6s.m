function theta = Ikine6s(T, config)
% Ikine6s
% 单组逆解接口。为兼容实验四旧代码而保留。
% 内部调用 Ikine6s_all，然后默认选第一组解。
% 对连续轨迹，推荐使用 SolveIKTrajectory + SelectIKSolution。

sols = Ikine6s_all(T, config);
if isempty(sols)
    error('Ikine6s:NoSolution', '目标位姿无逆解。');
end
theta = sols(:,1).';
end
