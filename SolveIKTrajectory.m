function [q, info] = SolveIKTrajectory(Cart_Traj, R_target, cfg)
% SolveIKTrajectory
% 对笛卡尔轨迹逐点逆解，并用“与上一时刻最接近”原则选解。
% 相比直接调用单组 Ikine6s，这种方法更适合连续轨迹，能减少关节跳变。

numPts = size(Cart_Traj,1);
q = zeros(numPts,6);

info.numSolutions = zeros(numPts,1);
info.selectedCost = zeros(numPts,1);
info.failedIndex = [];

qPrev = cfg.qSeed(:).';

for i = 1:numPts
    T_target = [R_target, Cart_Traj(i,:).'; 0 0 0 1];
    sols = Ikine6s_all(T_target, cfg.config);  % 6 x N

    if isempty(sols)
        info.failedIndex = [info.failedIndex; i];
        if i == 1
            error('第 1 个轨迹点无逆解，请检查木琴坐标、锤头偏置和末端姿态。');
        else
            warning('第 %d 个点无逆解，暂用上一帧关节角。', i);
            q(i,:) = qPrev;
            continue;
        end
    end

    sols = sols.'; % N x 6
    info.numSolutions(i) = size(sols,1);

    [qBest, costBest] = SelectIKSolution(sols, qPrev, cfg);
    q(i,:) = qBest;
    info.selectedCost(i) = costBest;
    qPrev = qBest;
end
end
